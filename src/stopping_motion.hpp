#pragma once

#include "motion_profile.hpp"

namespace swerve_mppi::detail {
inline double braking_duration(const VehicleState &start, const Twist2d &before, const Config &c) {
  double duration = std::max(std::hypot(before.vx, before.vy) / c.max_linear_decel_mps2,
                             std::abs(before.wz) / c.max_angular_decel_radps2);
  for (double speed : start.wheel_speeds)
    duration = std::max(duration, std::abs(speed) / c.max_wheel_accel_mps2);
  return duration;
}
inline double braking_duration(const VehicleState &start, const Config &c) {
  return braking_duration(start, Kinematics(c).forward(start.wheel_speeds, start.steering_angles), c);
}
// Hold/RequestMode may align after the complete proportional brake, using only
// the remaining tick time. A stopped threshold never authorizes rolling steering.
inline StepResult stopping_step(const VehicleState &start,
                                const std::array<double, 4> &steering_targets, const Config &c,
                                double dt) {
  StepResult out;
  out.state = start;
  out.steering_targets = steering_targets;
  const auto before = Kinematics(c).forward(start.wheel_speeds, start.steering_angles);
  const double duration = braking_duration(start, before, c);
  const double scale = duration > 0 ? std::max(0.0, 1 - dt / duration) : 0;
  for (std::size_t i = 0; i < 4; ++i)
    out.state.wheel_speeds[i] = start.wheel_speeds[i] * scale;
  out.state.velocity = Kinematics(c).forward(out.state.wheel_speeds, start.steering_angles);
  MotionProfile profile;
  profile.duration = profile.durations[2] = std::min(duration, dt);
  integrate_profile(out, before, profile, dt);
  if (is_stopped(start, c) && duration < dt) {
    const double limit = c.max_steer_rate_radps * (dt - duration);
    for (std::size_t i = 0; i < 4; ++i)
      out.state.steering_angles[i] +=
          std::clamp(steering_targets[i] - start.steering_angles[i], -limit, limit);
  }
  out.wheel_speed_targets = out.state.wheel_speeds;
  out.state.stamp_s += dt;
  out.state.time_in_mode_s += dt;
  return out;
}
} // namespace swerve_mppi::detail
