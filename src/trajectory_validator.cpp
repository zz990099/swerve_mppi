#include "swerve_mppi/trajectory_validator.hpp"
#include "validation.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swerve_mppi {
TrajectoryValidator::TrajectoryValidator(const Config &config) : config_(config) {
  validate(config_);
}
void TrajectoryValidator::require_compatible(const Config &config) const {
  validate(config);
  if (config.robot_radius_m != config_.robot_radius_m ||
      config.collision_margin_m != config_.collision_margin_m ||
      config.steering_limit_rad != config_.steering_limit_rad ||
      config.max_wheel_speed_mps != config_.max_wheel_speed_mps)
    throw std::invalid_argument("trajectory validator safety configuration differs from consumer");
}
void TrajectoryValidator::add(std::shared_ptr<const TrajectoryConstraint> constraint) {
  if (!constraint)
    throw std::invalid_argument("trajectory constraint must not be null");
  constraints_.push_back(std::move(constraint));
}
TrajectoryStatus TrajectoryValidator::check(const ControllerInput &input,
                                            const Trajectory &trajectory) const {
  if (!trajectory.valid || trajectory.poses.empty() || !detail::valid_input(input, config_) ||
      (!trajectory.sweep_margins_m.empty() &&
       trajectory.sweep_margins_m.size() + 1 != trajectory.poses.size()))
    return TrajectoryStatus::Invalid;
  for (const auto &pose : trajectory.poses)
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw))
      return TrajectoryStatus::Invalid;
  for (double margin : trajectory.sweep_margins_m)
    if (!std::isfinite(margin) || margin < 0)
      return TrajectoryStatus::Invalid;
  for (std::size_t i = 0; i < trajectory.poses.size(); ++i) {
    const double margin =
        i == 0 || trajectory.sweep_margins_m.empty() ? 0 : trajectory.sweep_margins_m[i - 1];
    const auto &from = trajectory.poses[i == 0 ? 0 : i - 1];
    const auto &to = trajectory.poses[i];
    const double dx = to.x - from.x, dy = to.y - from.y;
    const double length2 = dx * dx + dy * dy;
    for (const auto &obstacle : input.obstacles) {
      const double t =
          length2 > 1e-12
              ? std::clamp(((obstacle.x - from.x) * dx + (obstacle.y - from.y) * dy) / length2, 0.0,
                           1.0)
              : 0.0;
      const double clearance =
          std::hypot(from.x + t * dx - obstacle.x, from.y + t * dy - obstacle.y) - obstacle.radius -
          config_.robot_radius_m - config_.collision_margin_m - margin;
      if (!std::isfinite(clearance))
        return TrajectoryStatus::Invalid;
      if (clearance <= 0)
        return TrajectoryStatus::Collision;
    }
  }
  for (const auto &constraint : constraints_)
    if (!constraint->allows(input, trajectory))
      return TrajectoryStatus::Rejected;
  return TrajectoryStatus::Valid;
}
} // namespace swerve_mppi
