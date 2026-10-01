#include "swerve_mppi/critics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_mppi {
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
double clamp(double value, double low, double high) { return std::max(low, std::min(value, high)); }

struct PathMatch {
  double distance;
  double yaw;
};
PathMatch path_match(const Pose2d &pose, const std::vector<Pose2d> &path) {
  double nearest_squared = kInfinity;
  std::size_t nearest = 0;
  double nearest_t = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1].x, dy = path[i].y - path[i - 1].y;
    const double length2 = dx * dx + dy * dy;
    const double t =
        length2 > 1e-12
            ? clamp(((pose.x - path[i - 1].x) * dx + (pose.y - path[i - 1].y) * dy) / length2, 0, 1)
            : 0;
    const double x = pose.x - path[i - 1].x - t * dx, y = pose.y - path[i - 1].y - t * dy;
    const double distance2 = x * x + y * y;
    if (distance2 < nearest_squared) {
      nearest_squared = distance2;
      nearest = i;
      nearest_t = t;
    }
  }
  if (path.size() == 1)
    return {std::hypot(pose.x - path[0].x, pose.y - path[0].y), path[0].yaw};
  if (nearest == 0)
    return {kInfinity, path.front().yaw};
  // Compare squared distances while scanning; evaluate distance and body yaw
  // only for the nearest segment, not for every intermediate improvement.
  return {std::sqrt(nearest_squared),
          wrap_angle(path[nearest - 1].yaw +
                     nearest_t * angle_distance(path[nearest].yaw, path[nearest - 1].yaw))};
}

double segment_obstacle_cost(const Pose2d &previous, const Pose2d &next,
                             const ControllerInput &input, const Config &config) {
  double cost = 0.0;
  const double dx = next.x - previous.x;
  const double dy = next.y - previous.y;
  const double length2 = dx * dx + dy * dy;
  for (const auto &obstacle : input.obstacles) {
    const double t =
        length2 > 1e-12
            ? clamp(((obstacle.x - previous.x) * dx + (obstacle.y - previous.y) * dy) / length2,
                    0.0, 1.0)
            : 0.0;
    const double clearance =
        std::hypot(previous.x + t * dx - obstacle.x, previous.y + t * dy - obstacle.y) -
        obstacle.radius - config.robot_radius_m - config.collision_margin_m;
    if (clearance <= 0.0)
      return kInfinity;
    if (clearance < 0.5) {
      cost += config.clearance_weight / (clearance + 0.03);
    }
  }
  return cost;
}

class PathCritic final : public Critic {
public:
  explicit PathCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "PathDistance"; }
  double score(const ControllerInput &input, const Trajectory &trajectory) const override {
    if (input.reference_path.empty())
      return kInfinity;
    double cost = 0.0;
    for (std::size_t i = 1; i < trajectory.poses.size(); ++i) {
      const auto match = path_match(trajectory.poses[i], input.reference_path);
      cost += config_.dt_s * config_.path_weight * match.distance * match.distance;
      if (input.tracking && input.tracking->heading_policy == PathHeadingPolicy::FollowPath) {
        const double yaw = angle_distance(trajectory.poses[i].yaw, match.yaw);
        cost += config_.dt_s * config_.path_heading_weight * yaw * yaw;
      }
    }
    return cost;
  }

private:
  Config config_;
};
class ObstacleCritic final : public Critic {
public:
  explicit ObstacleCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "CircleObstacle"; }
  double score(const ControllerInput &input, const Trajectory &trajectory) const override {
    if (!std::isfinite(segment_obstacle_cost(trajectory.poses.front(), trajectory.poses.front(),
                                             input, config_)))
      return kInfinity;
    double cost = 0.0;
    for (std::size_t i = 1; i < trajectory.poses.size(); ++i)
      cost += config_.dt_s *
              segment_obstacle_cost(trajectory.poses[i - 1], trajectory.poses[i], input, config_);
    return cost;
  }

private:
  Config config_;
};
class GoalCritic final : public Critic {
public:
  explicit GoalCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "GoalPose"; }
  double score(const ControllerInput &input, const Trajectory &trajectory) const override {
    if (input.reference_path.empty())
      return kInfinity;
    const auto &goal = input.reference_path.back();
    const auto &pose = trajectory.final_state.pose;
    const double d = std::hypot(pose.x - goal.x, pose.y - goal.y);
    const double yaw = angle_distance(pose.yaw, goal.yaw);
    const bool terminal =
        !input.tracking || (input.tracking->terminal &&
                            input.tracking->remaining_length_m < config_.goal_slowdown_distance_m);
    double cost = config_.goal_weight * d * d + (terminal ? config_.yaw_weight * yaw * yaw : 0.0);
    if (input.tracking && terminal) {
      const auto &v = trajectory.final_state.velocity;
      cost += config_.goal_speed_weight * (v.vx * v.vx + v.vy * v.vy);
    }
    return cost;
  }

private:
  Config config_;
};
class EffortCritic final : public Critic {
public:
  explicit EffortCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "ControlEffort"; }
  double score(const ControllerInput &, const Trajectory &trajectory) const override {
    double cost = 0.0;
    for (const auto &u : trajectory.controls)
      cost += config_.dt_s * config_.effort_weight * (u.vx * u.vx + u.vy * u.vy + u.wz * u.wz);
    return cost;
  }

private:
  Config config_;
};
class SmoothnessCritic final : public Critic {
public:
  explicit SmoothnessCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "ControlChange"; }
  double score(const ControllerInput &input, const Trajectory &trajectory) const override {
    if (trajectory.active_controls.size() != trajectory.controls.size())
      return kInfinity;
    Control previous{input.vehicle.velocity.vx, input.vehicle.velocity.vy,
                     input.vehicle.velocity.wz};
    double cost = 0;
    for (std::size_t i = 0; i < trajectory.controls.size(); ++i) {
      if (!trajectory.active_controls[i])
        continue;
      const auto &u = trajectory.controls[i];
      const double vx = u.vx - previous.vx, vy = u.vy - previous.vy, wz = u.wz - previous.wz;
      cost += config_.smoothness_weight * (vx * vx + vy * vy + wz * wz);
      previous = u;
    }
    return cost;
  }

private:
  Config config_;
};
class SwitchCritic final : public Critic {
public:
  explicit SwitchCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "ModeSwitch"; }
  double score(const ControllerInput &, const Trajectory &trajectory) const override {
    return trajectory.branch.switches ? config_.switch_cost : 0.0;
  }

private:
  Config config_;
};
} // namespace
CriticManager::CriticManager(const Config &config) {
  validate(config);
  add(std::make_shared<PathCritic>(config));
  add(std::make_shared<ObstacleCritic>(config));
  add(std::make_shared<GoalCritic>(config));
  add(std::make_shared<EffortCritic>(config));
  add(std::make_shared<SwitchCritic>(config));
  add(std::make_shared<SmoothnessCritic>(config));
}
void CriticManager::add(std::shared_ptr<const Critic> critic) {
  if (!critic)
    throw std::invalid_argument("critic must not be null");
  critics_.push_back(std::move(critic));
}
double CriticManager::score(const ControllerInput &input, const Trajectory &trajectory) const {
  if (!trajectory.valid || trajectory.poses.empty())
    return kInfinity;
  double total = 0.0;
  for (const auto &critic : critics_) {
    const double part = critic->score(input, trajectory);
    if (!std::isfinite(part))
      return kInfinity;
    total += part;
    if (!std::isfinite(total))
      return kInfinity;
  }
  return total;
}
} // namespace swerve_mppi
