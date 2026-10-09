#include "swerve_mppi/planning/critics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "common/detail/spatial_index.hpp"
#include "safety/detail/validation.hpp"

namespace swerve_mppi
{
namespace detail
{
struct ScoringGeometry
{
  static SpatialIndex path_index(const std::vector<Pose2d> & path)
  {
    std::vector<Bounds> bounds;
    bounds.reserve(path.size() - 1);
    for (std::size_t i = 1; i < path.size(); ++i) {
      bounds.push_back(Bounds::segment(path[i - 1], path[i]));
    }
    return SpatialIndex(std::move(bounds));
  }
  static double path_curvature(const std::vector<Pose2d> & path)
  {
    double curvature = 0;
    Pose2d a = path.front(), b = a;
    bool have_segment = false;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const auto & d = path[i];
      const double bx = d.x - b.x, by = d.y - b.y;
      const double second = finite_geometry(std::hypot(bx, by));
      if (second <= 1e-9) {
        continue;
      }
      if (have_segment) {
        const double ax = b.x - a.x, ay = b.y - a.y;
        curvature = std::max(
          curvature, finite_geometry(
                       std::abs(geometry_turn(ax, ay, bx, by)) /
                       ((finite_geometry(std::hypot(ax, ay)) + second) / 2)));
      }
      a = b;
      b = d;
      have_segment = true;
    }
    return curvature;
  }
  explicit ScoringGeometry(const ControllerInput & input)
  : path(path_index(input.reference_path)),
    obstacles(obstacle_index(input.obstacles)),
    curvature(path_curvature(input.reference_path))
  {
  }
  SpatialIndex path;
  SpatialIndex obstacles;
  double curvature;
};
}  // namespace detail
namespace
{
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct PathMatch
{
  double distance;
  double yaw;
};
PathMatch path_match(
  const Pose2d & pose, const std::vector<Pose2d> & path,
  const detail::SpatialIndex * index = nullptr)
{
  double nearest_distance = kInfinity;
  std::size_t nearest = 0;
  double nearest_t = 0;
  const auto inspect = [&](std::size_t segment, double & best) {
    const auto i = segment + 1;
    const auto match = detail::segment_distance(pose, path[i - 1], path[i]);
    if (match.distance < best || (match.distance == best && (nearest == 0 || i < nearest))) {
      best = match.distance;
      nearest = i;
      nearest_t = match.fraction;
    }
  };
  if (path.size() == 1) {
    return {
      detail::finite_geometry(std::hypot(pose.x - path[0].x, pose.y - path[0].y)), path[0].yaw};
  }
  if (index) {
    index->nearest(pose, nearest_distance, inspect);
  } else {
    for (std::size_t i = 1; i < path.size(); ++i) {
      inspect(i - 1, nearest_distance);
    }
  }
  if (nearest == 0) {
    return {kInfinity, path.front().yaw};
  }
  return {
    nearest_distance, wrap_angle(
                        wrap_angle(path[nearest - 1].yaw) +
                        nearest_t * angle_distance(path[nearest].yaw, path[nearest - 1].yaw))};
}

double segment_obstacle_cost(
  const Pose2d & previous, const Pose2d & next, const ControllerInput & input,
  const Config & config, const detail::SpatialIndex * index = nullptr)
{
  double cost = 0;
  const double padding = detail::finite_geometry(config.robot_radius_m + config.collision_margin_m);
  const auto query = detail::Bounds::segment(previous, next, padding + .5);
  const auto inspect = [&](std::size_t i) {
    const auto & o = input.obstacles[i];
    if (
      !index && !query.overlaps(detail::Bounds::segment({o.x, o.y, 0}, {o.x, o.y, 0}, o.radius))) {
      return true;
    }
    const double clearance = detail::finite_geometry(
      detail::segment_distance({o.x, o.y, 0}, previous, next).distance - o.radius - padding);
    if (clearance <= 0) {
      cost = kInfinity;
      return false;
    }
    if (clearance < .5) {
      // Clearance is a distance-field property, independent of obstacle sampling density.
      cost = std::max(cost, config.clearance_weight / (clearance + .03));
    }
    return true;
  };
  if (index) {
    index->visit(query, inspect);
  } else {
    for (std::size_t i = 0; i < input.obstacles.size() && inspect(i); ++i) {
    }
  }
  return cost;
}

class PathCritic final : public Critic
{
public:
  explicit PathCritic(Config config, const detail::SpatialIndex * index = nullptr)
  : config_(config), index_(index)
  {
  }
  std::string_view name() const override { return "PathDistance"; }
  double score(const ControllerInput & input, const Trajectory & trajectory) const override
  {
    if (input.reference_path.empty()) {
      return kInfinity;
    }
    double cost = 0.0;
    for (std::size_t i = 1; i < trajectory.poses.size(); ++i) {
      const auto match = path_match(trajectory.poses[i], input.reference_path, index_);
      cost += config_.model_period_s * config_.path_weight * match.distance * match.distance;
      if (input.tracking && input.tracking->heading_policy == PathHeadingPolicy::FollowPath) {
        const double yaw = angle_distance(trajectory.poses[i].yaw, match.yaw);
        cost += config_.model_period_s * config_.path_heading_weight * yaw * yaw;
      }
    }
    return cost;
  }

private:
  Config config_;
  const detail::SpatialIndex * index_;
};
class ObstacleCritic final : public Critic
{
public:
  explicit ObstacleCritic(Config config, const detail::SpatialIndex * index = nullptr)
  : config_(config), index_(index)
  {
  }
  std::string_view name() const override { return "CircleObstacle"; }
  double score(const ControllerInput & input, const Trajectory & trajectory) const override
  {
    if (!std::isfinite(segment_obstacle_cost(
          trajectory.poses.front(), trajectory.poses.front(), input, config_, index_))) {
      return kInfinity;
    }
    double cost = 0.0;
    for (std::size_t i = 1; i < trajectory.poses.size(); ++i) {
      cost +=
        config_.model_period_s *
        segment_obstacle_cost(trajectory.poses[i - 1], trajectory.poses[i], input, config_, index_);
    }
    return cost;
  }

private:
  Config config_;
  const detail::SpatialIndex * index_;
};
class GoalCritic final : public Critic
{
public:
  explicit GoalCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "GoalPose"; }
  double score(const ControllerInput & input, const Trajectory & trajectory) const override
  {
    if (input.reference_path.empty()) {
      return kInfinity;
    }
    const auto & goal = input.reference_path.back();
    const auto & pose = trajectory.final_state.pose;
    const double d = std::hypot(pose.x - goal.x, pose.y - goal.y);
    const double yaw = angle_distance(pose.yaw, goal.yaw);
    const bool terminal =
      !input.tracking || (input.tracking->terminal &&
                          input.tracking->remaining_length_m < config_.goal_slowdown_distance_m);
    double cost = config_.goal_weight * d * d + (terminal ? config_.yaw_weight * yaw * yaw : 0.0);
    if (input.tracking && terminal) {
      const auto & v = trajectory.final_state.velocity;
      cost += config_.goal_speed_weight * (v.vx * v.vx + v.vy * v.vy);
    }
    return cost;
  }

private:
  Config config_;
};
class EffortCritic final : public Critic
{
public:
  explicit EffortCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "ControlEffort"; }
  double score(const ControllerInput &, const Trajectory & trajectory) const override
  {
    double cost = 0.0;
    for (const auto & u : trajectory.controls) {
      cost +=
        config_.model_period_s * config_.effort_weight * (u.vx * u.vx + u.vy * u.vy + u.wz * u.wz);
    }
    return cost;
  }

private:
  Config config_;
};
class SmoothnessCritic final : public Critic
{
public:
  explicit SmoothnessCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "ControlChange"; }
  double score(const ControllerInput & input, const Trajectory & trajectory) const override
  {
    if (trajectory.active_controls.size() != trajectory.controls.size()) {
      return kInfinity;
    }
    Control previous{
      input.vehicle.velocity.vx, input.vehicle.velocity.vy, input.vehicle.velocity.wz};
    double cost = 0;
    for (std::size_t i = 0; i < trajectory.controls.size(); ++i) {
      if (!trajectory.active_controls[i]) {
        continue;
      }
      const auto & u = trajectory.controls[i];
      const double vx = u.vx - previous.vx, vy = u.vy - previous.vy, wz = u.wz - previous.wz;
      cost += config_.smoothness_weight * (vx * vx + vy * vy + wz * wz);
      previous = u;
    }
    return cost;
  }

private:
  Config config_;
};
class SwitchCritic final : public Critic
{
public:
  explicit SwitchCritic(Config config) : config_(config) {}
  std::string_view name() const override { return "ModeSwitch"; }
  double score(const ControllerInput &, const Trajectory & trajectory) const override
  {
    return trajectory.branch.switches ? config_.switch_cost : 0.0;
  }

private:
  Config config_;
};
}  // namespace
CriticManager::CriticManager(
  const Config & config, std::shared_ptr<const TrajectoryValidator> validator)
: config_(config),
  validator_(validator ? std::move(validator) : std::make_shared<TrajectoryValidator>(config))
{
  validate(config);
  validator_->require_compatible(config);
  add(std::make_shared<PathCritic>(config));
  add(std::make_shared<ObstacleCritic>(config));
  add(std::make_shared<GoalCritic>(config));
  add(std::make_shared<EffortCritic>(config));
  add(std::make_shared<SwitchCritic>(config));
  add(std::make_shared<SmoothnessCritic>(config));
}
void CriticManager::add(std::shared_ptr<const Critic> critic)
{
  if (!critic) {
    throw std::invalid_argument("critic must not be null");
  }
  critics_.push_back(std::move(critic));
}
double CriticManager::score(const ControllerInput & input, const Trajectory & trajectory) const
{
  if (validator_->check(input, trajectory) != TrajectoryStatus::Valid) {
    return kInfinity;
  }
  try {
    double total = 0.0;
    for (const auto & critic : critics_) {
      const double part = critic->score(input, trajectory);
      if (!std::isfinite(part)) {
        return kInfinity;
      }
      total += part;
      if (!std::isfinite(total)) {
        return kInfinity;
      }
    }
    return total;
  } catch (const std::invalid_argument &) {
    return kInfinity;
  }
}
bool CriticManager::prepare(const ControllerInput & input)
{
  if (!detail::valid_input(input, config_)) {
    geometry_.reset();
    return false;
  }
  try {
    geometry_ = std::make_shared<const detail::ScoringGeometry>(input);
    return true;
  } catch (const std::invalid_argument &) {
    geometry_.reset();
    return false;
  }
}
double CriticManager::prepared_curvature() const { return geometry_->curvature; }
double CriticManager::score_prepared(
  const ControllerInput & input, const Trajectory & trajectory) const
{
  if (
    !geometry_ || validator_->check_indexed(input, trajectory, &geometry_->obstacles) !=
                    TrajectoryStatus::Valid) {
    return kInfinity;
  }
  try {
    double cost = PathCritic(config_, &geometry_->path).score(input, trajectory) +
                  ObstacleCritic(config_, &geometry_->obstacles).score(input, trajectory);
    for (std::size_t i = 2; i < critics_.size(); ++i) {
      cost += critics_[i]->score(input, trajectory);
      if (!std::isfinite(cost)) {
        return kInfinity;
      }
    }
    return std::isfinite(cost) ? cost : kInfinity;
  } catch (const std::invalid_argument &) {
    return kInfinity;
  }
}
}  // namespace swerve_mppi
