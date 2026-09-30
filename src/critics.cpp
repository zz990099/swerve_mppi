#include "swerve_mppi/critics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_mppi {
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
double clamp(double value, double low, double high) { return std::max(low, std::min(value, high)); }

double path_distance(const Pose2d &pose, const std::vector<Pose2d> &path) {
  double nearest = kInfinity;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    const double length2 = dx * dx + dy * dy;
    const double t =
        length2 > 1e-12
            ? clamp(((pose.x - path[i - 1].x) * dx + (pose.y - path[i - 1].y) * dy) / length2, 0.0,
                    1.0)
            : 0.0;
    nearest = std::min(
        nearest, std::hypot(pose.x - (path[i - 1].x + t * dx), pose.y - (path[i - 1].y + t * dy)));
  }
  return path.size() == 1 ? std::hypot(pose.x - path[0].x, pose.y - path[0].y) : nearest;
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
      const double d = path_distance(trajectory.poses[i], input.reference_path);
      cost += config_.dt_s * config_.path_weight * d * d;
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
    return config_.goal_weight * d * d + config_.yaw_weight * yaw * yaw;
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
