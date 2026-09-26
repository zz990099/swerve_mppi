#include "swerve_mppi/model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swerve_mppi {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double approach(double value, double target, double maximum_change) {
  return value + clamp(target - value, -maximum_change, maximum_change);
}

double linear_speed(const Twist2d & twist) {
  return std::hypot(twist.vx, twist.vy);
}

bool stopped(const Twist2d & twist, const Config & config) {
  return linear_speed(twist) <= config.stopped_linear_mps &&
         std::abs(twist.wz) <= config.stopped_angular_radps;
}

void integrate(Pose2d & pose, const Twist2d & velocity, double dt) {
  const double middle_yaw = pose.yaw + velocity.wz * dt * 0.5;
  pose.x += (velocity.vx * std::cos(middle_yaw) -
             velocity.vy * std::sin(middle_yaw)) * dt;
  pose.y += (velocity.vx * std::sin(middle_yaw) +
             velocity.vy * std::cos(middle_yaw)) * dt;
  pose.yaw = wrap_angle(pose.yaw + velocity.wz * dt);
}

bool finite(double x) { return std::isfinite(x); }
}  // namespace

double wrap_angle(double angle) { return std::remainder(angle, 2.0 * kPi); }

double angle_distance(double a, double b) { return wrap_angle(a - b); }

void validate(const Config & c) {
  const double positive[] = {
      c.wheelbase_m, c.track_m, c.robot_radius_m, c.max_wheel_speed_mps,
      c.max_steer_rate_radps, c.max_vx_mps, c.max_crab_speed_mps,
      c.max_spin_radps, c.max_yaw_rate_radps, c.min_turn_radius_m,
      c.max_linear_accel_mps2, c.max_linear_decel_mps2,
      c.max_angular_accel_radps2, c.steering_tolerance_rad,
      c.confirmation_timeout_s, c.dt_s, c.temperature};
  for (const double value : positive) {
    if (!finite(value) || value <= 0.0) {
      throw std::invalid_argument("positive, finite vehicle and MPPI parameters required");
    }
  }
  const double nonnegative[] = {
      c.collision_margin_m, c.stopped_linear_mps, c.stopped_angular_radps,
      c.minimum_mode_dwell_s, c.alignment_min_s,
      c.confirmation_prediction_s, c.switch_cost, c.switch_hysteresis,
      c.noise_v_mps, c.noise_w_radps, c.path_weight, c.goal_weight,
      c.yaw_weight, c.effort_weight, c.clearance_weight};
  for (const double value : nonnegative) {
    if (!finite(value) || value < 0.0) {
      throw std::invalid_argument("nonnegative, finite parameters required");
    }
  }
  if (c.horizon_steps < 2 || c.samples_per_branch < 1 || c.iterations < 1) {
    throw std::invalid_argument("MPPI horizon, sample count, and iterations invalid");
  }
}

bool DriveModel::feasible(const Control & u, DriveMode mode) const {
  if (!finite(u.vx) || !finite(u.vy) || !finite(u.wz)) return false;
  constexpr double tol = 1e-7;
  switch (mode) {
    case DriveMode::DualAckermann:
      return std::abs(u.vy) <= tol &&
             std::abs(u.vx) <= config_.max_vx_mps + tol &&
             std::abs(u.wz) <= config_.max_yaw_rate_radps + tol &&
             std::abs(u.wz) <= std::abs(u.vx) / config_.min_turn_radius_m + tol;
    case DriveMode::Spin:
      return std::abs(u.vx) <= tol && std::abs(u.vy) <= tol &&
             std::abs(u.wz) <= config_.max_spin_radps + tol;
    case DriveMode::Crab:
      return std::hypot(u.vx, u.vy) <= config_.max_crab_speed_mps + tol &&
             std::abs(u.wz) <= tol;
  }
  return false;
}

Control DriveModel::project(const Control & u, DriveMode mode) const {
  switch (mode) {
    case DriveMode::DualAckermann: {
      const double vx = clamp(u.vx, -config_.max_vx_mps, config_.max_vx_mps);
      const double limit = std::min(config_.max_yaw_rate_radps,
                                    std::abs(vx) / config_.min_turn_radius_m);
      return {vx, 0.0, clamp(u.wz, -limit, limit)};
    }
    case DriveMode::Spin:
      return {0.0, 0.0,
              clamp(u.wz, -config_.max_spin_radps, config_.max_spin_radps)};
    case DriveMode::Crab: {
      const double scale = std::min(1.0, config_.max_crab_speed_mps /
                                                (std::hypot(u.vx, u.vy) + kEpsilon));
      return {u.vx * scale, u.vy * scale, 0.0};
    }
  }
  return {};
}

std::array<double, 4> DriveModel::steering_for_mode(DriveMode mode) const {
  if (mode != DriveMode::Spin) return {};
  const double x[4] = {config_.wheelbase_m * 0.5, config_.wheelbase_m * 0.5,
                       -config_.wheelbase_m * 0.5, -config_.wheelbase_m * 0.5};
  const double y[4] = {config_.track_m * 0.5, -config_.track_m * 0.5,
                       config_.track_m * 0.5, -config_.track_m * 0.5};
  std::array<double, 4> targets{};
  for (int i = 0; i < 4; ++i) targets[i] = std::atan2(x[i], -y[i]);
  return targets;
}

StepResult DriveModel::step(const VehicleState & start, const Control & requested,
                             double dt) const {
  StepResult result;
  result.state = start;
  if (dt <= 0.0 || !std::isfinite(dt) ||
      !feasible(requested, start.actual_mode)) {
    result.valid = false;
    return result;
  }
  const Control u = project(requested, start.actual_mode);
  const double x[4] = {config_.wheelbase_m * 0.5, config_.wheelbase_m * 0.5,
                       -config_.wheelbase_m * 0.5, -config_.wheelbase_m * 0.5};
  const double y[4] = {config_.track_m * 0.5, -config_.track_m * 0.5,
                       config_.track_m * 0.5, -config_.track_m * 0.5};
  bool steering_ready = true;
  for (int i = 0; i < 4; ++i) {
    const double wx = u.vx - u.wz * y[i];
    const double wy = u.vy + u.wz * x[i];
    const double speed = std::hypot(wx, wy);
    if (speed > config_.max_wheel_speed_mps + kEpsilon) {
      result.valid = false;
      return result;
    }
    double angle = start.steering_angles[i];
    double signed_speed = 0.0;
    if (speed > kEpsilon) {
      angle = std::atan2(wy, wx);
      signed_speed = speed;
      if (std::abs(angle_distance(angle + kPi, start.steering_angles[i])) <
          std::abs(angle_distance(angle, start.steering_angles[i]))) {
        angle = wrap_angle(angle + kPi);
        signed_speed = -speed;
      }
    }
    result.steering_targets[i] = angle;
    result.wheel_speed_targets[i] = signed_speed;
    if (std::abs(angle_distance(angle, start.steering_angles[i])) >
        config_.steering_tolerance_rad) {
      steering_ready = false;
    }
  }

  auto & state = result.state;
  const double linear_rate = steering_ready ? config_.max_linear_accel_mps2
                                             : config_.max_linear_decel_mps2;
  const Twist2d target = steering_ready ? Twist2d{u.vx, u.vy, u.wz} : Twist2d{};
  const double dvx = target.vx - start.velocity.vx;
  const double dvy = target.vy - start.velocity.vy;
  const double norm = std::hypot(dvx, dvy);
  const double scale = norm > kEpsilon ?
      std::min(1.0, linear_rate * dt / norm) : 1.0;
  state.velocity.vx = start.velocity.vx + scale * dvx;
  state.velocity.vy = start.velocity.vy + scale * dvy;
  state.velocity.wz = approach(start.velocity.wz, target.wz,
                                config_.max_angular_accel_radps2 * dt);
  if (stopped(state.velocity, config_) && !steering_ready) {
    state.velocity = {};
    for (int i = 0; i < 4; ++i) {
      state.steering_angles[i] = wrap_angle(start.steering_angles[i] +
          clamp(angle_distance(result.steering_targets[i], start.steering_angles[i]),
                -config_.max_steer_rate_radps * dt,
                config_.max_steer_rate_radps * dt));
    }
  }
  // During braking the steering stays fixed. Report wheel targets for the
  // acceleration-limited body velocity, rather than the raw sampled velocity.
  for (int i = 0; i < 4; ++i) {
    const double wx = state.velocity.vx - state.velocity.wz * y[i];
    const double wy = state.velocity.vy + state.velocity.wz * x[i];
    const double actual_angle = steering_ready ? result.steering_targets[i] :
                                                   start.steering_angles[i];
    const double signed_speed = wx * std::cos(actual_angle) +
                                wy * std::sin(actual_angle);
    result.wheel_speed_targets[i] = steering_ready ? signed_speed : 0.0;
    state.wheel_speeds[i] = signed_speed;
  }
  integrate(state.pose, state.velocity, dt);
  state.stamp_s += dt;
  state.time_in_mode_s += dt;
  return result;
}

double TransitionModel::rollout(VehicleState & state, DriveMode target_mode,
                                std::size_t & steps_used,
                                std::size_t max_steps,
                                std::vector<Pose2d> * trace) const {
  const std::size_t start_step = steps_used;
  while (!stopped(state.velocity, config_)) {
    if (steps_used >= max_steps) return -1.0;
    const StepResult next = model_.step(state, {}, config_.dt_s);
    if (!next.valid) return -1.0;
    state = next.state;
    ++steps_used;
    if (trace) trace->push_back(state.pose);
  }
  state.velocity = {};
  state.wheel_speeds.fill(0.0);
  const auto angles = model_.steering_for_mode(target_mode);
  const std::size_t min_align_steps = static_cast<std::size_t>(
      std::ceil(config_.alignment_min_s / config_.dt_s));
  std::size_t align_steps = 0;
  while (true) {
    bool aligned = true;
    for (int i = 0; i < 4; ++i) {
      const double delta = angle_distance(angles[i], state.steering_angles[i]);
      const double alternate = angle_distance(angles[i] + kPi,
                                               state.steering_angles[i]);
      if (std::min(std::abs(delta), std::abs(alternate)) >
          config_.steering_tolerance_rad) aligned = false;
    }
    if (aligned && align_steps >= min_align_steps) break;
    if (steps_used >= max_steps) return -1.0;
    for (int i = 0; i < 4; ++i) {
      const double delta = angle_distance(angles[i], state.steering_angles[i]);
      const double alternate = angle_distance(angles[i] + kPi,
                                               state.steering_angles[i]);
      const double best = std::abs(alternate) < std::abs(delta) ? alternate : delta;
      state.steering_angles[i] = wrap_angle(state.steering_angles[i] +
          clamp(best, -config_.max_steer_rate_radps * config_.dt_s,
                config_.max_steer_rate_radps * config_.dt_s));
    }
    state.stamp_s += config_.dt_s;
    ++steps_used;
    ++align_steps;
    if (trace) trace->push_back(state.pose);
  }
  const std::size_t confirm_steps = static_cast<std::size_t>(
      std::ceil(config_.confirmation_prediction_s / config_.dt_s));
  if (confirm_steps > max_steps - steps_used) return -1.0;
  state.stamp_s += confirm_steps * config_.dt_s;
  steps_used += confirm_steps;
  if (trace) {
    for (std::size_t i = 0; i < confirm_steps; ++i) trace->push_back(state.pose);
  }
  state.actual_mode = target_mode;
  state.time_in_mode_s = 0.0;
  return (steps_used - start_step) * config_.dt_s;
}

}  // namespace swerve_mppi
