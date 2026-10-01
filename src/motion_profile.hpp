#pragma once

#include "swerve_mppi/model.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace swerve_mppi::detail {
// A piecewise linear body-twist ramp. Split at the linear speed minimum and
// angular zero crossing so braking time cannot be reused as acceleration time.
struct MotionProfile {
  std::array<double, 4> fractions{0, 0, 0, 1};
  std::array<double, 3> durations{};
  double duration = 0;
};
inline Twist2d interpolate(const Twist2d &a, const Twist2d &b, double f) {
  return {a.vx + f * (b.vx - a.vx), a.vy + f * (b.vy - a.vy), a.wz + f * (b.wz - a.wz)};
}
inline double minimum_fraction(double a, double b, double dx, double dy) {
  const double length2 = dx * dx + dy * dy;
  return length2 > 0 ? std::clamp(-(a * dx + b * dy) / length2, 0.0, 1.0) : 0;
}
inline MotionProfile motion_profile(const Twist2d &before, const VehicleState &after,
                                    const VehicleState &start, const Config &c) {
  MotionProfile p;
  const auto &end = after.velocity;
  p.fractions[1] = minimum_fraction(before.vx, before.vy, end.vx - before.vx, end.vy - before.vy);
  p.fractions[2] = minimum_fraction(before.wz, 0, end.wz - before.wz, 0);
  std::sort(p.fractions.begin(), p.fractions.end());
  double joint_time = 0;
  for (std::size_t j = 0; j < 4; ++j)
    joint_time = std::max(
        {joint_time,
         std::abs(after.wheel_speeds[j] - start.wheel_speeds[j]) / c.max_wheel_accel_mps2,
         std::abs(after.steering_angles[j] - start.steering_angles[j]) / c.max_steer_rate_radps});
  for (std::size_t i = 0; i < 3; ++i) {
    const auto a = interpolate(before, end, p.fractions[i]);
    const auto b = interpolate(before, end, p.fractions[i + 1]);
    const double linear = std::hypot(b.vx, b.vy) < std::hypot(a.vx, a.vy) ? c.max_linear_decel_mps2
                                                                          : c.max_linear_accel_mps2;
    const double angular =
        std::abs(b.wz) < std::abs(a.wz) ? c.max_angular_decel_radps2 : c.max_angular_accel_radps2;
    p.durations[i] =
        std::max({std::hypot(b.vx - a.vx, b.vy - a.vy) / linear, std::abs(b.wz - a.wz) / angular,
                  (p.fractions[i + 1] - p.fractions[i]) * joint_time});
    p.duration += p.durations[i];
  }
  return p;
}
inline void integrate_constant(Pose2d &pose, const Twist2d &v, double dt) {
  const double a = v.wz * dt;
  const double sinc = std::abs(a) < 1e-4 ? 1 - a * a / 6 + a * a * a * a / 120 : std::sin(a) / a;
  const double cosc =
      std::abs(a) < 1e-4 ? a / 2 - a * a * a / 24 + a * a * a * a * a / 720 : (1 - std::cos(a)) / a;
  const double dx = dt * (sinc * v.vx - cosc * v.vy);
  const double dy = dt * (cosc * v.vx + sinc * v.vy);
  pose.x += std::cos(pose.yaw) * dx - std::sin(pose.yaw) * dy;
  pose.y += std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  pose.yaw = wrap_angle(pose.yaw + a);
}
inline void integrate_ramp(Pose2d &pose, const Twist2d &a, const Twist2d &b, double dt,
                           double &error, double &acceleration) {
  if (dt <= 0)
    return;
  const double speed = std::max(std::hypot(a.vx, a.vy), std::hypot(b.vx, b.vy));
  const double bound =
      std::hypot(b.vx - a.vx, b.vy - a.vy) / dt + speed * std::max(std::abs(a.wz), std::abs(b.wz));
  acceleration = std::max(acceleration, bound);
  // Commuting SE(2) twists (including straight ramps and fixed-curvature
  // braking) integrate exactly using their time average.
  if (a.wz * b.vx == b.wz * a.vx && a.wz * b.vy == b.wz * a.vy) {
    integrate_constant(pose, interpolate(a, b, .5), dt);
    return;
  }
  constexpr std::size_t subdivisions = 8;
  const double h = dt / subdivisions;
  const double yaw = pose.yaw;
  for (std::size_t i = 0; i < subdivisions; ++i) {
    const double f = (i + .5) / subdivisions;
    const auto v = interpolate(a, b, f);
    // Exact yaw of this linear angular ramp at the quadrature midpoint.
    const double heading = yaw + dt * (a.wz * f + (b.wz - a.wz) * f * f / 2);
    pose.x += (std::cos(heading) * v.vx - std::sin(heading) * v.vy) * h;
    pose.y += (std::sin(heading) * v.vx + std::cos(heading) * v.vy) * h;
  }
  pose.yaw = wrap_angle(yaw + (a.wz + b.wz) * dt / 2);
  // Lipschitz world velocity: each midpoint interval contributes <= A*h^2/4.
  error += bound * dt * dt / (4 * subdivisions);
}
inline void integrate_profile(StepResult &out, const Twist2d &before, const MotionProfile &p,
                              double dt) {
  double acceleration = 0;
  for (std::size_t i = 0; i < 3; ++i)
    integrate_ramp(out.state.pose, interpolate(before, out.state.velocity, p.fractions[i]),
                   interpolate(before, out.state.velocity, p.fractions[i + 1]), p.durations[i],
                   out.integration_error_m, acceleration);
  const double remaining = std::max(0.0, dt - p.duration);
  const auto &v = out.state.velocity;
  acceleration = std::max(acceleration, std::hypot(v.vx, v.vy) * std::abs(v.wz));
  integrate_constant(out.state.pose, v, remaining);
  // A curve with bounded world acceleration deviates from its endpoint chord
  // by <= A*dt^2/8, including interior excursions during a signed reversal.
  const bool straight_monotonic = before.wz == 0 && v.wz == 0 &&
                                  before.vx * v.vy == before.vy * v.vx &&
                                  before.vx * v.vx + before.vy * v.vy >= 0;
  out.sweep_margin_m =
      (straight_monotonic ? 0 : acceleration * dt * dt / 8) + out.integration_error_m;
}
inline void append_motion(const StepResult &step, std::vector<Pose2d> *poses,
                          std::vector<double> *margins, double &position_error) {
  if (poses)
    poses->push_back(step.state.pose);
  if (margins)
    margins->push_back(position_error + step.sweep_margin_m);
  position_error += step.integration_error_m;
}
} // namespace swerve_mppi::detail
