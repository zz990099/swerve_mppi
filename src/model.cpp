#include "swerve_mppi/model.hpp"
#include "swerve_mppi/feedback.hpp"

#include "drive_interpolation.hpp"
#include "motion_profile.hpp"
#include "stopping_motion.hpp"
#include "validation.hpp"
#include <algorithm>
#include <cmath>

namespace swerve_mppi {
namespace {
constexpr double kEpsilon = 1e-9;
// Split the velocity segment at its minimum speed. Traversing the braking
// part consumes deceleration time before any remaining acceleration time.
double braking_fraction(double ix, double iy, double fx, double fy) {
  const double dx = fx - ix, dy = fy - iy;
  const double squared = dx * dx + dy * dy;
  return squared > 0.0 ? std::clamp(-(ix * dx + iy * dy) / squared, 0.0, 1.0) : 0.0;
}
double velocity_fraction(double ix, double iy, double fx, double fy, double accel, double decel,
                         double dt) {
  const double length = std::hypot(fx - ix, fy - iy);
  if (length < kEpsilon)
    return 1.0;
  const double braking = length * braking_fraction(ix, iy, fx, fy);
  const double brake_time = braking / decel;
  const double distance = dt <= brake_time ? dt * decel : braking + (dt - brake_time) * accel;
  return std::min(1.0, distance / length);
}
double velocity_change_time(double ix, double iy, double fx, double fy, double accel,
                            double decel) {
  const double fraction = braking_fraction(ix, iy, fx, fy);
  return std::hypot(fx - ix, fy - iy) * (fraction / decel + (1.0 - fraction) / accel);
}
} // namespace

bool is_stopped(const VehicleState &state, const Config &c) {
  if (std::hypot(state.velocity.vx, state.velocity.vy) > c.stopped_linear_mps ||
      std::abs(state.velocity.wz) > c.stopped_angular_radps)
    return false;
  return std::all_of(state.wheel_speeds.begin(), state.wheel_speeds.end(),
                     [&](double speed) { return std::abs(speed) <= c.stopped_wheel_speed_mps; });
}

DriveModel::DriveModel(const Config &config) : config_(config), kinematics_(config) {}
bool DriveModel::feasible(const Control &u, DriveMode mode) const {
  if (!std::isfinite(u.vx) || !std::isfinite(u.vy) || !std::isfinite(u.wz))
    return false;
  constexpr double tol = 1e-7;
  bool admissible = false;
  switch (mode) {
  case DriveMode::DualAckermann:
    admissible = std::abs(u.vy) <= tol && std::abs(u.vx) <= config_.max_vx_mps + tol &&
                 std::abs(u.wz) <= config_.max_yaw_rate_radps + tol &&
                 std::abs(u.wz) <= std::abs(u.vx) / config_.min_turn_radius_m + tol;
    break;
  case DriveMode::Spin:
    admissible = std::abs(u.vx) <= tol && std::abs(u.vy) <= tol &&
                 std::abs(u.wz) <= config_.max_spin_radps + tol;
    break;
  case DriveMode::Crab:
    admissible =
        std::hypot(u.vx, u.vy) <= config_.max_crab_speed_mps + tol && std::abs(u.wz) <= tol;
    break;
  }
  if (!admissible)
    return false;
  const auto wheels = kinematics_.inverse(u, {});
  return wheels.valid && std::all_of(wheels.speeds.begin(), wheels.speeds.end(), [&](double speed) {
           return std::abs(speed) <= config_.max_wheel_speed_mps + tol;
         });
}

Control DriveModel::project(const Control &u, DriveMode mode) const {
  Control out;
  if (!std::isfinite(u.vx) || !std::isfinite(u.vy) || !std::isfinite(u.wz))
    return out;
  switch (mode) {
  case DriveMode::DualAckermann: {
    out.vx = std::clamp(u.vx, -config_.max_vx_mps, config_.max_vx_mps);
    const double limit =
        std::min(config_.max_yaw_rate_radps, std::abs(out.vx) / config_.min_turn_radius_m);
    out.wz = std::clamp(u.wz, -limit, limit);
    break;
  }
  case DriveMode::Spin:
    out.wz = std::clamp(u.wz, -config_.max_spin_radps, config_.max_spin_radps);
    break;
  case DriveMode::Crab: {
    const double scale =
        std::min(1.0, config_.max_crab_speed_mps / (std::hypot(u.vx, u.vy) + kEpsilon));
    out = {u.vx * scale, u.vy * scale, 0.0};
    break;
  }
  }
  const auto wheels = kinematics_.inverse(out, {});
  double maximum = 0.0;
  for (double speed : wheels.speeds)
    maximum = std::max(maximum, std::abs(speed));
  const double scale =
      maximum > config_.max_wheel_speed_mps ? config_.max_wheel_speed_mps / maximum : 1.0;
  return {out.vx * scale, out.vy * scale, out.wz * scale};
}

std::array<double, 4> DriveModel::steering_for_mode(DriveMode mode,
                                                    const std::array<double, 4> &current) const {
  return mode == DriveMode::Spin ? kinematics_.inverse({0.0, 0.0, 1.0}, current).angles
                                 : std::array<double, 4>{};
}

std::array<double, 4> DriveModel::steering_for_entry(DriveMode mode, const Control &intent,
                                                     const std::array<double, 4> &current) const {
  const auto projected = project(intent, mode);
  if (std::hypot(projected.vx, projected.vy) < kEpsilon && std::abs(projected.wz) < kEpsilon)
    return steering_for_mode(mode, current);
  return kinematics_.inverse(projected, current).angles;
}

StepResult DriveModel::step(const VehicleState &start, const Control &u, double dt) const {
  StepResult out;
  out.state = start;
  if (!std::isfinite(dt) || dt <= 0.0 ||
      check_feedback(start, config_).status != FeedbackStatus::Valid ||
      !feasible(u, start.actual_mode)) {
    out.valid = false;
    return out;
  }
  const auto wheels = kinematics_.inverse(u, start.steering_angles);
  if (!wheels.valid) {
    out.valid = false;
    return out;
  }
  bool ready = true;
  for (std::size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(start.wheel_speeds[i]) ||
        std::abs(start.wheel_speeds[i]) > config_.max_wheel_speed_mps + kEpsilon ||
        std::abs(wheels.speeds[i]) > config_.max_wheel_speed_mps + kEpsilon) {
      out.valid = false;
      return out;
    }
    if (std::abs(wheels.angles[i] - start.steering_angles[i]) > config_.drive_steering_limit_rad)
      ready = false;
  }
  out.steering_targets =
      ready || is_stopped(start, config_) ? wheels.angles : start.steering_angles;
  out.aligning = !ready;
  const bool driving = ready && (std::hypot(u.vx, u.vy) >= kEpsilon || std::abs(u.wz) >= kEpsilon);
  if (!driving) {
    auto stopped = detail::stopping_step(start, out.steering_targets, config_, dt);
    stopped.aligning = !ready;
    return stopped;
  }
  auto &state = out.state;
  auto target = wheels.speeds;
  const Twist2d initial = kinematics_.forward(start.wheel_speeds, start.steering_angles);
  auto target_fraction = [&]() {
    const auto desired = kinematics_.forward(target, wheels.angles);
    double fraction = std::min(
        velocity_fraction(initial.vx, initial.vy, desired.vx, desired.vy,
                          config_.max_linear_accel_mps2, config_.max_linear_decel_mps2, dt),
        velocity_fraction(initial.wz, 0.0, desired.wz, 0.0, config_.max_angular_accel_radps2,
                          config_.max_angular_decel_radps2, dt));
    for (std::size_t i = 0; i < 4; ++i) {
      const double delta = std::abs(target[i] - start.wheel_speeds[i]);
      if (delta > kEpsilon)
        fraction = std::min(fraction, config_.max_wheel_accel_mps2 * dt / delta);
      const double steering_delta = std::abs(wheels.angles[i] - start.steering_angles[i]);
      if (steering_delta > kEpsilon)
        fraction = std::min(fraction, config_.max_steer_rate_radps * dt / steering_delta);
    }
    return fraction;
  };
  double fraction = target_fraction();
  // Steering changes the encoder-derived twist too. Limit the joint step as a
  // whole, rather than checking acceleration at fixed steering angles only.
  detail::MotionProfile profile;
  for (std::size_t attempt = 0;; ++attempt) {
    for (std::size_t i = 0; i < 4; ++i) {
      state.wheel_speeds[i] =
          start.wheel_speeds[i] + fraction * (target[i] - start.wheel_speeds[i]);
      state.steering_angles[i] =
          start.steering_angles[i] + fraction * (wheels.angles[i] - start.steering_angles[i]);
    }
    state.velocity = kinematics_.forward(state.wheel_speeds, state.steering_angles);
    profile = detail::motion_profile(initial, state, start, config_);
    const bool module_consistent = kinematics_.max_module_residual(
                                       state.wheel_speeds, state.steering_angles, state.velocity) <=
                                   config_.drive_kinematic_tolerance_mps + 1e-9;
    const bool speed_ok = detail::drive_speed_admissible(start, state, config_);
    if (module_consistent && speed_ok &&
        detail::drive_rates_admissible(start, state, config_, dt) &&
        profile.duration <= dt + 1e-12 &&
        velocity_change_time(initial.vx, initial.vy, state.velocity.vx, state.velocity.vy,
                             config_.max_linear_accel_mps2,
                             config_.max_linear_decel_mps2) <= dt + 1e-9 &&
        velocity_change_time(initial.wz, 0.0, state.velocity.wz, 0.0,
                             config_.max_angular_accel_radps2,
                             config_.max_angular_decel_radps2) <= dt + 1e-9)
      break;
    if (attempt == 60) {
      out.valid = false;
      return out;
    }
    if (!speed_ok) {
      // Reducing steering alone can stall a turn at an exact speed cap. Reserve
      // a small rolling-speed margin while retaining the intended geometry.
      for (double &speed : target)
        speed *= .99;
      fraction = target_fraction();
    } else
      fraction *= 0.5;
  }
  out.steering_targets = state.steering_angles;
  out.wheel_speed_targets = state.wheel_speeds;
  detail::integrate_drive(out, start, initial, config_, dt);
  state.stamp_s += dt;
  state.time_in_mode_s += dt;
  return out;
}

} // namespace swerve_mppi
