#pragma once

#include <algorithm>
#include <cmath>

#include "safety/detail/validation.hpp"
#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi::detail
{
inline bool within_body_limits(const Twist2d & v, DriveMode mode, const Config & c)
{
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
struct EncoderSample
{
  Twist2d velocity;
  Twist2d derivative;  // Derivative with respect to the tick fraction.
};
inline EncoderSample encoder_sample(
  const VehicleState & start, const VehicleState & end, const Config & c, double f)
{
  EncoderSample out;
  const double x[] = {c.wheelbase_m / 2, c.wheelbase_m / 2, -c.wheelbase_m / 2, -c.wheelbase_m / 2};
  const double y[] = {c.track_m / 2, -c.track_m / 2, c.track_m / 2, -c.track_m / 2};
  const double moment = c.wheelbase_m * c.wheelbase_m + c.track_m * c.track_m;
  for (std::size_t i = 0; i < 4; ++i) {
    const double ds = end.wheel_speeds[i] - start.wheel_speeds[i];
    const double da = end.steering_angles[i] - start.steering_angles[i];
    const double angle = start.steering_angles[i] + f * da;
    const double speed = start.wheel_speeds[i] + f * ds;
    const double cosine = std::cos(angle), sine = std::sin(angle);
    const double vx = speed * cosine, vy = speed * sine;
    const double dx = ds * cosine - speed * da * sine;
    const double dy = ds * sine + speed * da * cosine;
    out.velocity.vx += vx / 4;
    out.velocity.vy += vy / 4;
    out.velocity.wz += (x[i] * vy - y[i] * vx) / moment;
    out.derivative.vx += dx / 4;
    out.derivative.vy += dy / 4;
    out.derivative.wz += (x[i] * dy - y[i] * dx) / moment;
  }
  return out;
}
// Certify absolute speeds over the entire affine joint interval. Triangle
// bounds handle ordinary capped Crab/Spin motion cheaply. Adaptive chord
// enclosures and derivative-sign certificates cover cancellation near a cap. A
// finite sampling grid alone cannot certify unsampled peaks; bounded exhaustion
// fails closed.
inline bool drive_speed_admissible(
  const VehicleState & start, const VehicleState & end, const Config & c)
{
  const auto first = encoder_sample(start, end, c, 0);
  const auto last = encoder_sample(start, end, c, 1);
  if (
    !within_body_limits(first.velocity, start.actual_mode, c) ||
    !within_body_limits(last.velocity, start.actual_mode, c)) {
    return false;
  }
  if (start.steering_angles == end.steering_angles) {
    return true;  // The body twist is affine; each mode's speed set is convex.
  }
  double second = 0, rolling = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    const double ds = end.wheel_speeds[i] - start.wheel_speeds[i];
    const double da = end.steering_angles[i] - start.steering_angles[i];
    const double speed = std::max(std::abs(start.wheel_speeds[i]), std::abs(end.wheel_speeds[i]));
    second += std::hypot(2 * ds * da, speed * da * da) / 4;
    rolling += speed / 4;
  }
  constexpr double tolerance = 1e-9;
  const double radius = std::hypot(c.wheelbase_m / 2, c.track_m / 2);
  const auto mode = start.actual_mode;
  const double linear_limit = mode == DriveMode::Crab ? c.max_crab_speed_mps : c.max_vx_mps;
  const double angular_limit = mode == DriveMode::Spin ? c.max_spin_radps : c.max_yaw_rate_radps;
  const bool linear_triangle = mode == DriveMode::Spin || rolling <= linear_limit + tolerance;
  const bool angular_triangle =
    mode == DriveMode::Crab || rolling / radius <= angular_limit + tolerance;
  if (linear_triangle && angular_triangle) {
    return true;
  }
  std::size_t budget = 4096;
  auto certify = [&](
                   auto && self, double a, const EncoderSample & va, double b,
                   const EncoderSample & vb, std::size_t depth) -> bool {
    if (budget == 0) {
      return false;
    }
    --budget;
    const double middle = (a + b) / 2, h = b - a;
    const auto vm = encoder_sample(start, end, c, middle);
    if (!within_body_limits(vm.velocity, mode, c)) {
      return false;
    }
    auto scalar = [&](double from, double to, double derivative, double bound, double limit) {
      const double peak = std::max(std::abs(from), std::abs(to));
      const bool monotonic = std::abs(derivative) >= bound * h / 2;
      return peak + (monotonic ? 0 : bound * h * h / 8) <= limit + tolerance;
    };
    const bool linear =
      linear_triangle ||
      (mode == DriveMode::Crab
         ? std::max(
             std::hypot(va.velocity.vx, va.velocity.vy),
             std::hypot(vb.velocity.vx, vb.velocity.vy)) +
               second * h * h / 8 <=
             linear_limit + tolerance
         : scalar(va.velocity.vx, vb.velocity.vx, vm.derivative.vx, second, linear_limit));
    const bool angular =
      angular_triangle ||
      scalar(va.velocity.wz, vb.velocity.wz, vm.derivative.wz, second / radius, angular_limit);
    if (linear && angular) {
      return true;
    }
    return depth < 14 && self(self, a, va, middle, vm, depth + 1) &&
           self(self, middle, vm, b, vb, depth + 1);
  };
  return certify(certify, 0, first, 1, last, 0);
}
// Affine joint targets reach their endpoint at the end of the whole Drive tick.
// Check pointwise rates, including both sides of a signed reversal. Moving
// steering uses a conservative derivative bound for the encoder velocity field.
inline bool drive_rates_admissible(
  const VehicleState & start, const VehicleState & end, const Config & c, double dt)
{
  const auto before = Kinematics(c).forward(start.wheel_speeds, start.steering_angles);
  bool fixed = true;
  double second = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    const double ds = end.wheel_speeds[i] - start.wheel_speeds[i];
    const double da = end.steering_angles[i] - start.steering_angles[i];
    if (
      std::abs(ds) > c.max_wheel_accel_mps2 * dt + 1e-9 ||
      std::abs(da) > c.max_steer_rate_radps * dt + 1e-9) {
      return false;
    }
    fixed = fixed && da == 0;
    const double speed = std::max(std::abs(start.wheel_speeds[i]), std::abs(end.wheel_speeds[i]));
    second += std::hypot(2 * ds * da, speed * da * da) / (4 * dt * dt);
  }
  if (!fixed) {
    const double radius = std::hypot(c.wheelbase_m / 2, c.track_m / 2);
    double linear = 0, angular = 0;
    for (double f : {0.0, .5, 1.0}) {
      const auto sample = encoder_sample(start, end, c, f);
      linear = std::max(linear, std::hypot(sample.derivative.vx, sample.derivative.vy) / dt);
      angular = std::max(angular, std::abs(sample.derivative.wz) / dt);
    }
    // Every point is at most dt/4 from a derivative sample. This analytic
    // second-derivative bound encloses all unsampled rates, including
    // cancellations.
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
           dt * rate(
                  before.vx, before.vy, end.velocity.vx, end.velocity.vy, c.max_linear_accel_mps2,
                  c.max_linear_decel_mps2) +
             1e-9 &&
         std::abs(end.velocity.wz - before.wz) <= dt * rate(
                                                         before.wz, 0, end.velocity.wz, 0,
                                                         c.max_angular_accel_radps2,
                                                         c.max_angular_decel_radps2) +
                                                    1e-9;
}
inline bool drive_interpolation_admissible(
  const VehicleState & start, const VehicleState & end, const Config & c, double dt)
{
  return drive_speed_admissible(start, end, c) && drive_rates_admissible(start, end, c, dt);
}
// Shared by the protocol supervisor and the checked actuator reference.
inline bool drive_targets_admissible(
  const VehicleState & start, const std::array<double, 4> & angles,
  const std::array<double, 4> & speeds, const Config & c)
{
  if (!valid_vehicle(start, c) || !valid_steering(angles, c)) {
    return false;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    const double delta = std::abs(angles[i] - start.steering_angles[i]);
    if (
      !std::isfinite(speeds[i]) || std::abs(speeds[i]) > c.max_wheel_speed_mps + 1e-9 ||
      delta > c.drive_steering_limit_rad + 1e-9 || delta > c.max_steer_rate_radps * c.dt_s + 1e-9) {
      return false;
    }
  }
  const Kinematics kinematics(c);
  VehicleState end = start;
  end.steering_angles = angles;
  end.wheel_speeds = speeds;
  end.velocity = kinematics.forward(speeds, angles);
  if (
    !within_body_limits(end.velocity, start.actual_mode, c) ||
    !drive_interpolation_admissible(start, end, c, c.dt_s) ||
    kinematics.max_module_residual(speeds, angles, end.velocity) >
      c.drive_kinematic_tolerance_mps + 1e-9) {
    return false;
  }
  const auto projected =
    DriveModel(c).project({end.velocity.vx, end.velocity.vy, end.velocity.wz}, start.actual_mode);
  double maximum_speed = 0;
  for (double speed : speeds) {
    maximum_speed = std::max(maximum_speed, std::abs(speed));
  }
  const double linear_error = maximum_speed * c.drive_steering_limit_rad + 1e-7;
  const double radius = std::hypot(c.wheelbase_m / 2, c.track_m / 2);
  return std::hypot(end.velocity.vx - projected.vx, end.velocity.vy - projected.vy) <=
           linear_error &&
         std::abs(end.velocity.wz - projected.wz) <= linear_error / radius;
}
}  // namespace swerve_mppi::detail
