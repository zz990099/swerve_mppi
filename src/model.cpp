#include "swerve_mppi/model.hpp"

#include "validation.hpp"
#include <algorithm>
#include <cmath>

namespace swerve_mppi {
namespace {
constexpr double kEpsilon = 1e-9;
void integrate(Pose2d &pose, const Twist2d &v, double dt) {
  const double a = v.wz * dt;
  double dx = v.vx * dt, dy = v.vy * dt;
  if (std::abs(v.wz) > kEpsilon) {
    const double s = std::sin(a) / v.wz, c = (1.0 - std::cos(a)) / v.wz;
    dx = s * v.vx - c * v.vy;
    dy = c * v.vx + s * v.vy;
  }
  pose.x += std::cos(pose.yaw) * dx - std::sin(pose.yaw) * dy;
  pose.y += std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  pose.yaw = wrap_angle(pose.yaw + a);
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

StepResult DriveModel::step(const VehicleState &start, const Control &u, double dt) const {
  StepResult out;
  out.state = start;
  if (!std::isfinite(dt) || dt <= 0.0 || !detail::valid_vehicle(start, config_) ||
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
    if (std::abs(wheels.angles[i] - start.steering_angles[i]) > config_.steering_tolerance_rad)
      ready = false;
  }
  out.steering_targets =
      ready || is_stopped(start, config_) ? wheels.angles : start.steering_angles;
  out.aligning = !ready;
  auto &state = out.state;
  if (!ready && is_stopped(start, config_)) {
    for (std::size_t i = 0; i < 4; ++i) {
      state.steering_angles[i] +=
          std::clamp(wheels.angles[i] - start.steering_angles[i],
                     -config_.max_steer_rate_radps * dt, config_.max_steer_rate_radps * dt);
    }
  }
  const std::array<double, 4> target = ready ? wheels.speeds : std::array<double, 4>{};
  const Twist2d initial = kinematics_.forward(start.wheel_speeds, start.steering_angles);
  const Twist2d desired = kinematics_.forward(target, state.steering_angles);
  const bool slowing = std::hypot(desired.vx, desired.vy) < std::hypot(initial.vx, initial.vy);
  const double linear_rate =
      slowing ? config_.max_linear_decel_mps2 : config_.max_linear_accel_mps2;
  const double angular_rate = std::abs(desired.wz) < std::abs(initial.wz)
                                  ? config_.max_angular_decel_radps2
                                  : config_.max_angular_accel_radps2;
  const double delta_linear = std::hypot(desired.vx - initial.vx, desired.vy - initial.vy);
  const double delta_angular = std::abs(desired.wz - initial.wz);
  double fraction = 1.0;
  if (delta_linear > kEpsilon)
    fraction = std::min(fraction, linear_rate * dt / delta_linear);
  if (delta_angular > kEpsilon)
    fraction = std::min(fraction, angular_rate * dt / delta_angular);
  for (std::size_t i = 0; i < 4; ++i) {
    const double delta = std::abs(target[i] - start.wheel_speeds[i]);
    if (delta > kEpsilon)
      fraction = std::min(fraction, config_.max_wheel_accel_mps2 * dt / delta);
  }
  for (std::size_t i = 0; i < 4; ++i) {
    state.wheel_speeds[i] = start.wheel_speeds[i] + fraction * (target[i] - start.wheel_speeds[i]);
  }
  state.velocity = kinematics_.forward(state.wheel_speeds, state.steering_angles);
  out.wheel_speed_targets = state.wheel_speeds;
  integrate(state.pose, state.velocity, dt);
  state.stamp_s += dt;
  state.time_in_mode_s += dt;
  return out;
}

} // namespace swerve_mppi
