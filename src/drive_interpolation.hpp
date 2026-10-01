#pragma once

#include "swerve_mppi/model.hpp"
#include <algorithm>
#include <cmath>

namespace swerve_mppi::detail {
inline bool within_body_limits(const Twist2d &v, DriveMode mode, const Config &c) {
  constexpr double tolerance = 1e-9;
  switch (mode) {
  case DriveMode::DualAckermann:
    return std::abs(v.vx) <= c.max_vx_mps + tolerance &&
           std::abs(v.wz) <= c.max_yaw_rate_radps + tolerance;
  case DriveMode::Crab:
    return std::hypot(v.vx, v.vy) <= c.max_crab_speed_mps + tolerance;
  case DriveMode::Spin:
    return std::abs(v.wz) <= c.max_spin_radps + tolerance;
  }
  return false;
}
// Affine joint targets reach their endpoint at the end of the whole Drive tick.
// Check pointwise rates, including both sides of a signed reversal. Moving
// steering uses a conservative derivative bound for the encoder velocity field.
inline bool drive_interpolation_admissible(const VehicleState &start, const VehicleState &end,
                                           const Config &c, double dt) {
  const auto before = Kinematics(c).forward(start.wheel_speeds, start.steering_angles);
  bool fixed = true;
  double second = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    const double ds = end.wheel_speeds[i] - start.wheel_speeds[i];
    const double da = end.steering_angles[i] - start.steering_angles[i];
    if (std::abs(ds) > c.max_wheel_accel_mps2 * dt + 1e-9 ||
        std::abs(da) > c.max_steer_rate_radps * dt + 1e-9)
      return false;
    fixed = fixed && da == 0;
    const double speed = std::max(std::abs(start.wheel_speeds[i]), std::abs(end.wheel_speeds[i]));
    second += std::hypot(2 * ds * da, speed * da * da) / (4 * dt * dt);
  }
  if (!fixed) {
    const double radius = std::hypot(c.wheelbase_m / 2, c.track_m / 2);
    double linear = 0, angular = 0;
    const double x[] = {c.wheelbase_m / 2, c.wheelbase_m / 2, -c.wheelbase_m / 2,
                        -c.wheelbase_m / 2};
    const double y[] = {c.track_m / 2, -c.track_m / 2, c.track_m / 2, -c.track_m / 2};
    for (double f : {0.0, .5, 1.0}) {
      double vx = 0, vy = 0, wz = 0;
      for (std::size_t i = 0; i < 4; ++i) {
        const double ds = end.wheel_speeds[i] - start.wheel_speeds[i];
        const double da = end.steering_angles[i] - start.steering_angles[i];
        const double angle = start.steering_angles[i] + f * da;
        const double speed = start.wheel_speeds[i] + f * ds;
        const double wx = (ds * std::cos(angle) - speed * da * std::sin(angle)) / dt;
        const double wy = (ds * std::sin(angle) + speed * da * std::cos(angle)) / dt;
        vx += wx / 4;
        vy += wy / 4;
        wz += (x[i] * wy - y[i] * wx) / (4 * radius * radius);
      }
      linear = std::max(linear, std::hypot(vx, vy));
      angular = std::max(angular, std::abs(wz));
    }
    // Every point is at most dt/4 from a derivative sample. This analytic
    // second-derivative bound encloses all unsampled rates, including cancellations.
    return linear + second * dt / 4 <=
               std::min(c.max_linear_accel_mps2, c.max_linear_decel_mps2) + 1e-9 &&
           angular + second * dt / (4 * radius) <=
               std::min(c.max_angular_accel_radps2, c.max_angular_decel_radps2) + 1e-9;
  }
  auto rate = [](double x, double y, double fx, double fy, double accel, double decel) {
    const double dx = fx - x, dy = fy - y;
    const double squared = dx * dx + dy * dy;
    const double minimum = squared > 0 ? std::clamp(-(x * dx + y * dy) / squared, 0.0, 1.0) : 0;
    return minimum == 0 ? accel : minimum == 1 ? decel : std::min(accel, decel);
  };
  return std::hypot(end.velocity.vx - before.vx, end.velocity.vy - before.vy) <=
             dt * rate(before.vx, before.vy, end.velocity.vx, end.velocity.vy,
                       c.max_linear_accel_mps2, c.max_linear_decel_mps2) +
                 1e-9 &&
         std::abs(end.velocity.wz - before.wz) <= dt * rate(before.wz, 0, end.velocity.wz, 0,
                                                            c.max_angular_accel_radps2,
                                                            c.max_angular_decel_radps2) +
                                                      1e-9;
}
} // namespace swerve_mppi::detail
