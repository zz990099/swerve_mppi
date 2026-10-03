#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi::detail
{
// A piecewise linear body-twist ramp. Split at the linear speed minimum and
// angular zero crossing so braking time cannot be reused as acceleration time.
struct MotionProfile
{
  std::array<double, 4> fractions{0, 0, 0, 1};
  std::array<double, 3> durations{};
  double duration = 0;
};
inline Twist2d interpolate(const Twist2d & a, const Twist2d & b, double f)
{
  return {a.vx + f * (b.vx - a.vx), a.vy + f * (b.vy - a.vy), a.wz + f * (b.wz - a.wz)};
}
inline double minimum_fraction(double a, double b, double dx, double dy)
{
  const double length2 = dx * dx + dy * dy;
  return length2 > 0 ? std::clamp(-(a * dx + b * dy) / length2, 0.0, 1.0) : 0;
}
inline MotionProfile motion_profile(
  const Twist2d & before, const VehicleState & after, const VehicleState & start, const Config & c)
{
  MotionProfile p;
  const auto & end = after.velocity;
  p.fractions[1] = minimum_fraction(before.vx, before.vy, end.vx - before.vx, end.vy - before.vy);
  p.fractions[2] = minimum_fraction(before.wz, 0, end.wz - before.wz, 0);
  std::sort(p.fractions.begin(), p.fractions.end());
  double joint_time = 0;
  for (std::size_t j = 0; j < 4; ++j) {
    joint_time = std::max(
      {joint_time, std::abs(after.wheel_speeds[j] - start.wheel_speeds[j]) / c.max_wheel_accel_mps2,
       std::abs(after.steering_angles[j] - start.steering_angles[j]) / c.max_steer_rate_radps});
  }
  for (std::size_t i = 0; i < 3; ++i) {
    const auto a = interpolate(before, end, p.fractions[i]);
    const auto b = interpolate(before, end, p.fractions[i + 1]);
    const double linear = std::hypot(b.vx, b.vy) < std::hypot(a.vx, a.vy) ? c.max_linear_decel_mps2
                                                                          : c.max_linear_accel_mps2;
    const double angular =
      std::abs(b.wz) < std::abs(a.wz) ? c.max_angular_decel_radps2 : c.max_angular_accel_radps2;
    p.durations[i] = std::max(
      {std::hypot(b.vx - a.vx, b.vy - a.vy) / linear, std::abs(b.wz - a.wz) / angular,
       (p.fractions[i + 1] - p.fractions[i]) * joint_time});
    p.duration += p.durations[i];
  }
  return p;
}
inline void integrate_constant(Pose2d & pose, const Twist2d & v, double dt)
{
  const double a = v.wz * dt;
  const double sinc = std::abs(a) < 1e-4 ? 1 - a * a / 6 + a * a * a * a / 120 : std::sin(a) / a;
  const double cosc =
    std::abs(a) < 1e-4 ? a / 2 - a * a * a / 24 + a * a * a * a * a / 720 : (1 - std::cos(a)) / a;
  const double dx = dt * (sinc * v.vx - cosc * v.vy);
  const double dy = dt * (cosc * v.vx + sinc * v.vy);
  const double yaw = wrap_angle(pose.yaw);
  pose.x += std::cos(yaw) * dx - std::sin(yaw) * dy;
  pose.y += std::sin(yaw) * dx + std::cos(yaw) * dy;
  pose.yaw = wrap_angle(yaw + a);
}
inline void integrate_ramp(
  Pose2d & pose, const Twist2d & a, const Twist2d & b, double dt, double & error,
  double & acceleration)
{
  if (dt <= 0) {
    return;
  }
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
  const double yaw = wrap_angle(pose.yaw);
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
inline void integrate_profile(
  StepResult & out, const Twist2d & before, const MotionProfile & p, double dt)
{
  double acceleration = 0;
  for (std::size_t i = 0; i < 3; ++i) {
    integrate_ramp(
      out.state.pose, interpolate(before, out.state.velocity, p.fractions[i]),
      interpolate(before, out.state.velocity, p.fractions[i + 1]), p.durations[i],
      out.integration_error_m, acceleration);
  }
  const double remaining = std::max(0.0, dt - p.duration);
  const auto & v = out.state.velocity;
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
// Integral of an affine rolling speed at an affine steering angle, from 0 to f.
// Small-angle moments avoid cancellation; yaw is integrated independently of
// the positional midpoint quadrature so no heading error accumulates
// downstream.
inline std::array<double, 2> wheel_integral(
  double speed, double ds, double angle, double da, double f)
{
  const double z = da * f, z2 = z * z;
  double sinc, cosc, first_cos, first_sin;
  if (std::abs(z) < .01) {
    sinc = 1 - z2 / 6 + z2 * z2 / 120 - z2 * z2 * z2 / 5040;
    cosc = z * (.5 - z2 / 24 + z2 * z2 / 720 - z2 * z2 * z2 / 40320);
    first_cos = .5 - z2 / 8 + z2 * z2 / 144 - z2 * z2 * z2 / 5760;
    first_sin = z * (1.0 / 3 - z2 / 30 + z2 * z2 / 840 - z2 * z2 * z2 / 45360);
  } else {
    sinc = std::sin(z) / z;
    cosc = (1 - std::cos(z)) / z;
    first_cos = sinc - cosc / z;
    first_sin = (sinc - std::cos(z)) / z;
  }
  const double x = speed * f * sinc + ds * f * f * first_cos;
  const double y = speed * f * cosc + ds * f * f * first_sin;
  return {std::cos(angle) * x - std::sin(angle) * y, std::sin(angle) * x + std::cos(angle) * y};
}
inline void integrate_drive(
  StepResult & out, const VehicleState & start, const Twist2d & before, const Config & c, double dt)
{
  const auto & end = out.state;
  if (start.steering_angles == end.steering_angles) {
    MotionProfile p;
    p.durations[2] = p.duration = dt;
    integrate_profile(out, before, p, dt);
    return;
  }
  const double x[] = {c.wheelbase_m / 2, c.wheelbase_m / 2, -c.wheelbase_m / 2, -c.wheelbase_m / 2};
  const double y[] = {c.track_m / 2, -c.track_m / 2, c.track_m / 2, -c.track_m / 2};
  const double radius = std::hypot(x[0], y[0]), moment = 4 * radius * radius;
  double derivative = 0, speed_bound = 0, angular_deviation = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    const double ds = end.wheel_speeds[i] - start.wheel_speeds[i];
    const double da = end.steering_angles[i] - start.steering_angles[i];
    const double speed = std::max(std::abs(start.wheel_speeds[i]), std::abs(end.wheel_speeds[i]));
    derivative += std::hypot(ds, speed * da) / (4 * dt);
    speed_bound += speed / 4;
    angular_deviation += (2 * std::abs(ds * da) + speed * da * da) / (32 * radius);
  }
  const double acceleration =
    derivative +
    speed_bound * (std::max(std::abs(before.wz), std::abs(end.velocity.wz)) + angular_deviation);
  auto yaw_at = [&](double f) {
    double yaw = wrap_angle(start.pose.yaw);
    for (std::size_t i = 0; i < 4; ++i) {
      const auto v = wheel_integral(
        start.wheel_speeds[i], end.wheel_speeds[i] - start.wheel_speeds[i],
        start.steering_angles[i], end.steering_angles[i] - start.steering_angles[i], f);
      yaw += dt * (x[i] * v[1] - y[i] * v[0]) / moment;
    }
    return yaw;
  };
  constexpr std::size_t subdivisions = 8;
  const double h = dt / subdivisions;
  for (std::size_t j = 0; j < subdivisions; ++j) {
    const double f = (j + .5) / subdivisions;
    double vx = 0, vy = 0;
    for (std::size_t i = 0; i < 4; ++i) {
      const double speed =
        start.wheel_speeds[i] + f * (end.wheel_speeds[i] - start.wheel_speeds[i]);
      const double angle =
        start.steering_angles[i] + f * (end.steering_angles[i] - start.steering_angles[i]);
      vx += speed * std::cos(angle) / 4;
      vy += speed * std::sin(angle) / 4;
    }
    const double yaw = yaw_at(f);
    out.state.pose.x += h * (std::cos(yaw) * vx - std::sin(yaw) * vy);
    out.state.pose.y += h * (std::sin(yaw) * vx + std::cos(yaw) * vy);
  }
  out.state.pose.yaw = wrap_angle(yaw_at(1));
  out.integration_error_m = acceleration * dt * dt / (4 * subdivisions);
  out.sweep_margin_m = acceleration * dt * dt / 8 + out.integration_error_m;
}
inline void append_motion(
  const StepResult & step, std::vector<Pose2d> * poses, std::vector<double> * margins,
  double & position_error)
{
  if (poses) {
    poses->push_back(step.state.pose);
  }
  if (margins) {
    margins->push_back(position_error + step.sweep_margin_m);
  }
  position_error += step.integration_error_m;
}
}  // namespace swerve_mppi::detail
