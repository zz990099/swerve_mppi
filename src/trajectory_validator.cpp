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
      config.max_wheel_speed_mps != config_.max_wheel_speed_mps ||
      config.max_path_points != config_.max_path_points ||
      config.max_obstacles != config_.max_obstacles ||
      config.feedback_linear_tolerance_mps != config_.feedback_linear_tolerance_mps ||
      config.feedback_angular_tolerance_radps != config_.feedback_angular_tolerance_radps)
    throw std::invalid_argument("trajectory validator safety configuration differs from consumer");
}
void TrajectoryValidator::add(std::shared_ptr<const TrajectoryConstraint> constraint) {
  if (!constraint)
    throw std::invalid_argument("trajectory constraint must not be null");
  constraints_.push_back(std::move(constraint));
}
TrajectoryStatus TrajectoryValidator::check(const ControllerInput &input,
                                            const Trajectory &trajectory) const {
  if (!trajectory.valid || trajectory.poses.empty() ||
      trajectory.poses.size() > 4097 || // Absolute public trace cap, including the initial pose.
      !detail::valid_input(input, config_) ||
      (!trajectory.sweep_margins_m.empty() &&
       trajectory.sweep_margins_m.size() + 1 != trajectory.poses.size()))
    return TrajectoryStatus::Invalid;
  for (const auto &pose : trajectory.poses)
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw))
      return TrajectoryStatus::Invalid;
  for (double margin : trajectory.sweep_margins_m)
    if (!std::isfinite(margin) || margin < 0)
      return TrajectoryStatus::Invalid;
  const auto &current = input.vehicle.pose;
  const auto &initial = trajectory.poses.front();
  constexpr double anchor_tolerance = 1e-9;
  const double yaw_error = angle_distance(initial.yaw, current.yaw);
  if (!std::isfinite(yaw_error) ||
      std::hypot(initial.x - current.x, initial.y - current.y) > anchor_tolerance ||
      std::abs(yaw_error) > anchor_tolerance)
    return TrajectoryStatus::Invalid;
  // Check the exact measured footprint even when the anchor differs by roundoff.
  for (const auto &obstacle : input.obstacles) {
    const double clearance = std::hypot(current.x - obstacle.x, current.y - obstacle.y) -
                             obstacle.radius - config_.robot_radius_m - config_.collision_margin_m;
    if (!std::isfinite(clearance))
      return TrajectoryStatus::Invalid;
    if (clearance <= 0)
      return TrajectoryStatus::Collision;
  }
  for (std::size_t i = 0; i < trajectory.poses.size(); ++i) {
    const double margin =
        i == 0 || trajectory.sweep_margins_m.empty() ? 0 : trajectory.sweep_margins_m[i - 1];
    const auto &from = trajectory.poses[i == 0 ? 0 : i - 1];
    const auto &to = trajectory.poses[i];
    const double dx = to.x - from.x, dy = to.y - from.y;
    const double length2 = dx * dx + dy * dy;
    for (const auto &obstacle : input.obstacles) {
      const double t =
          length2 > 0
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
