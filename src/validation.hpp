#pragma once
#include "swerve_mppi/config.hpp"
#include "swerve_mppi/types.hpp"
#include <cmath>
namespace swerve_mppi::detail {
inline bool valid_vehicle(const VehicleState &vehicle, const Config &config) {
  if ((vehicle.actual_mode != DriveMode::DualAckermann && vehicle.actual_mode != DriveMode::Spin &&
       vehicle.actual_mode != DriveMode::Crab) ||
      vehicle.stamp_s < 0.0 || !std::isfinite(vehicle.stamp_s) || !std::isfinite(vehicle.pose.x) ||
      !std::isfinite(vehicle.pose.y) || !std::isfinite(vehicle.pose.yaw) ||
      !std::isfinite(vehicle.velocity.vx) || !std::isfinite(vehicle.velocity.vy) ||
      !std::isfinite(vehicle.velocity.wz) || !std::isfinite(vehicle.time_in_mode_s) ||
      vehicle.time_in_mode_s < 0.0)
    return false;
  for (double angle : vehicle.steering_angles) {
    if (!std::isfinite(angle) || std::abs(angle) > config.steering_limit_rad)
      return false;
  }
  for (double speed : vehicle.wheel_speeds) {
    if (!std::isfinite(speed) || std::abs(speed) > config.max_wheel_speed_mps + 1e-9)
      return false;
  }
  return true;
}
inline bool valid_input(const ControllerInput &input, const Config &config) {
  if (input.reference_path.empty() || !valid_vehicle(input.vehicle, config))
    return false;
  for (const auto &waypoint : input.reference_path) {
    if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) || !std::isfinite(waypoint.yaw))
      return false;
  }
  for (const auto &obstacle : input.obstacles) {
    if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
        !std::isfinite(obstacle.radius) || obstacle.radius < 0.0)
      return false;
  }
  return true;
}
} // namespace swerve_mppi::detail
