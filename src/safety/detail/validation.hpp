#pragma once
#include <cmath>

#include "swerve_mppi/common/config.hpp"
#include "swerve_mppi/common/types.hpp"
#include "swerve_mppi/feedback/feedback.hpp"
namespace swerve_mppi::detail
{
inline bool valid_action(Action action)
{
  return action == Action::Drive || action == Action::Brake || action == Action::RequestMode ||
         action == Action::Hold || action == Action::SafeStop;
}
inline bool valid_mode(DriveMode mode)
{
  return mode == DriveMode::DualAckermann || mode == DriveMode::Spin || mode == DriveMode::Crab;
}
inline bool valid_steering(const std::array<double, 4> & angles, const Config & config)
{
  for (double angle : angles) {
    if (!std::isfinite(angle) || std::abs(angle) > config.steering_limit_rad) {
      return false;
    }
  }
  return true;
}
inline bool steering_aligned(
  const std::array<double, 4> & measured, const std::array<double, 4> & targets,
  const Config & config)
{
  for (std::size_t i = 0; i < measured.size(); ++i) {
    if (std::abs(measured[i] - targets[i]) > config.steering_tolerance_rad) {
      return false;
    }
  }
  return true;
}
inline bool valid_vehicle(const VehicleState & vehicle, const Config & config)
{
  if (
    (vehicle.actual_mode != DriveMode::DualAckermann && vehicle.actual_mode != DriveMode::Spin &&
     vehicle.actual_mode != DriveMode::Crab) ||
    vehicle.stamp_s < 0.0 || !std::isfinite(vehicle.stamp_s) || !std::isfinite(vehicle.pose.x) ||
    !std::isfinite(vehicle.pose.y) || !std::isfinite(vehicle.pose.yaw) ||
    !std::isfinite(vehicle.velocity.vx) || !std::isfinite(vehicle.velocity.vy) ||
    !std::isfinite(vehicle.velocity.wz) || !std::isfinite(vehicle.time_in_mode_s) ||
    vehicle.time_in_mode_s < 0.0) {
    return false;
  }
  for (double angle : vehicle.steering_angles) {
    if (!std::isfinite(angle) || std::abs(angle) > config.steering_limit_rad) {
      return false;
    }
  }
  for (double speed : vehicle.wheel_speeds) {
    if (!std::isfinite(speed) || std::abs(speed) > config.max_wheel_speed_mps + 1e-9) {
      return false;
    }
  }
  return true;
}
inline bool valid_input(const ControllerInput & input, const Config & config)
{
  if (
    input.reference_path.size() > config.max_path_points ||
    input.obstacles.size() > config.max_obstacles) {
    return false;
  }
  if (
    input.heading_policy != PathHeadingPolicy::FollowPath &&
    input.heading_policy != PathHeadingPolicy::GoalOnly) {
    return false;
  }
  if (input.tracking) {
    const auto & t = *input.tracking;
    if (
      !std::isfinite(t.goal.x) || !std::isfinite(t.goal.y) || !std::isfinite(t.goal.yaw) ||
      !std::isfinite(t.remaining_length_m) || t.remaining_length_m < 0 ||
      !std::isfinite(t.speed_limit_mps) || t.speed_limit_mps < 0 ||
      (t.heading_policy != PathHeadingPolicy::FollowPath &&
       t.heading_policy != PathHeadingPolicy::GoalOnly)) {
      return false;
    }
  }
  if (
    input.reference_path.empty() ||
    check_model_feedback(input.vehicle, config).status != FeedbackStatus::Valid) {
    return false;
  }
  for (const auto & waypoint : input.reference_path) {
    if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) || !std::isfinite(waypoint.yaw)) {
      return false;
    }
  }
  for (const auto & obstacle : input.obstacles) {
    if (
      !std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) || !std::isfinite(obstacle.radius) ||
      obstacle.radius < 0.0) {
      return false;
    }
  }
  return true;
}
}  // namespace swerve_mppi::detail
