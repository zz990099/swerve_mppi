#include "swerve_mppi/model/model.hpp"

#include <algorithm>
#include <cmath>

#include "model/detail/nominal_motion.hpp"
#include "safety/detail/validation.hpp"
#include "swerve_mppi/feedback/feedback.hpp"

namespace swerve_mppi
{
namespace
{
constexpr double kEpsilon = 1e-9;
}

bool is_stopped(const VehicleState & state, const Config & c)
{
  if (
    std::hypot(state.velocity.vx, state.velocity.vy) > c.stopped_linear_mps ||
    std::abs(state.velocity.wz) > c.stopped_angular_radps) {
    return false;
  }
  return std::all_of(state.wheel_speeds.begin(), state.wheel_speeds.end(), [&](double speed) {
    return std::abs(speed) <= c.stopped_wheel_speed_mps;
  });
}

DriveModel::DriveModel(const Config & config) : config_(config), kinematics_(config) {}
bool DriveModel::feasible(const Control & u, DriveMode mode) const
{
  if (!std::isfinite(u.vx) || !std::isfinite(u.vy) || !std::isfinite(u.wz)) {
    return false;
  }
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
  if (!admissible) {
    return false;
  }
  const auto capped = bounded(u);
  if (
    std::abs(capped.vx - u.vx) > tol || std::abs(capped.vy - u.vy) > tol ||
    std::abs(capped.wz - u.wz) > tol)
    return false;
  const auto wheels = kinematics_.inverse(u, {});
  return wheels.valid && std::all_of(wheels.speeds.begin(), wheels.speeds.end(), [&](double speed) {
           return std::abs(speed) <= config_.max_wheel_speed_mps + tol;
         });
}

Control DriveModel::project(const Control & u, DriveMode mode) const
{
  Control out;
  if (!std::isfinite(u.vx) || !std::isfinite(u.vy) || !std::isfinite(u.wz)) {
    return out;
  }
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
  out = bounded(out);
  const auto wheels = kinematics_.inverse(out, {});
  double maximum = 0.0;
  for (double speed : wheels.speeds) {
    maximum = std::max(maximum, std::abs(speed));
  }
  const double scale =
    maximum > config_.max_wheel_speed_mps ? config_.max_wheel_speed_mps / maximum : 1.0;
  return {out.vx * scale, out.vy * scale, out.wz * scale};
}

std::array<double, 4> DriveModel::steering_for_mode(
  DriveMode mode, const std::array<double, 4> & current) const
{
  return mode == DriveMode::Spin ? kinematics_.inverse({0.0, 0.0, 1.0}, current).angles
                                 : std::array<double, 4>{};
}

std::array<double, 4> DriveModel::steering_for_entry(
  DriveMode mode, const Control & intent, const std::array<double, 4> & current) const
{
  const double scale = std::max({std::abs(intent.vx), std::abs(intent.vy), std::abs(intent.wz)});
  if (scale == 0) return steering_for_mode(mode, current);
  // Normalize the raw legal entry, preserving curvature without planner projection.
  return kinematics_.inverse({intent.vx / scale, intent.vy / scale, intent.wz / scale}, current)
    .angles;
}

Control DriveModel::bounded(const Control & u) const
{
  const double scale = std::max(
    {1.0, std::hypot(u.vx, u.vy) / config_.chassis_max_linear_speed_mps,
     std::abs(u.wz) / config_.chassis_max_angular_speed_radps});
  return {u.vx / scale, u.vy / scale, u.wz / scale};
}
ChassisPrediction DriveModel::seed(const VehicleState & s) const
{
  ChassisPrediction memory;
  memory.limited_velocity = s.velocity;
  memory.commanded_angles = s.steering_angles;
  for (std::size_t i = 0; i < 4; ++i)
    memory.commanded_wheel_radps[i] = s.wheel_speeds[i] / config_.wheel_radius_m;
  return memory;
}
ChassisPrediction DriveModel::alignment_seed(
  const VehicleState & s, const std::array<double, 4> & angles) const
{
  auto memory = seed(s);
  memory.phase = TransitionPhase::Braking;
  memory.alignment = angles;
  memory.transition_start_s = s.stamp_s;
  return memory;
}
StepResult DriveModel::step(const VehicleState & s, const Control & u, double dt) const
{
  return step(s, u, dt, seed(s));
}
StepResult DriveModel::step(
  const VehicleState & start, const Control & u, double dt, const ChassisPrediction & memory) const
{
  StepResult out;
  out.state = start;
  out.prediction = memory;
  auto & m = out.prediction;
  if (
    !std::isfinite(dt) || dt <= 0 || dt / config_.chassis_period_s > 1024 ||
    check_model_feedback(start, config_).status != FeedbackStatus::Valid || start.mode_fault ||
    !std::isfinite(u.vx) || !std::isfinite(u.vy) || !std::isfinite(u.wz) ||
    (start.actual_mode == DriveMode::DualAckermann && (u.vy != 0 || (u.vx == 0 && u.wz != 0))) ||
    (start.actual_mode == DriveMode::Spin && (u.vx != 0 || u.vy != 0)) ||
    (start.actual_mode == DriveMode::Crab && u.wz != 0) ||
    !detail::valid_steering(m.commanded_angles, config_) ||
    !detail::valid_steering(m.alignment, config_) || !std::isfinite(m.limited_velocity.vx) ||
    !std::isfinite(m.limited_velocity.vy) || !std::isfinite(m.limited_velocity.wz) ||
    !std::isfinite(m.transition_start_s) || !std::isfinite(m.aligned_since_s) ||
    (m.phase != TransitionPhase::Stable && m.phase != TransitionPhase::Braking &&
     m.phase != TransitionPhase::Aligning)) {
    out.valid = false;
    return out;
  }
  for (double speed : m.commanded_wheel_radps) {
    if (
      !std::isfinite(speed) ||
      std::abs(speed) > config_.max_wheel_speed_mps / config_.wheel_radius_m + kEpsilon) {
      out.valid = false;
      return out;
    }
  }
  const auto velocity = bounded(u);
  auto targets = [&](const Control & intent) {
    auto w = kinematics_.inverse(intent, out.state.steering_angles);
    for (double & speed : w.speeds) speed /= config_.wheel_radius_m;
    double scale = 1;
    for (double speed : w.speeds)
      scale =
        std::max(scale, std::abs(speed) / (config_.max_wheel_speed_mps / config_.wheel_radius_m));
    for (double & speed : w.speeds) speed /= scale;
    return w;
  };
  auto approach = [](auto & values, const auto & target, double delta) {
    for (std::size_t i = 0; i < 4; ++i)
      values[i] += std::clamp(target[i] - values[i], -delta, delta);
  };
  // Each substep observes the previous targets, updates command mechanics, and
  // applies the new targets at its endpoint. This is an ideal tracking model.
  const std::size_t ticks =
    static_cast<std::size_t>(std::ceil(dt / config_.chassis_period_s - 1e-12));
  const double h = dt / std::max(ticks, std::size_t{1});
  double length = 0, largest_speed_jump = 0, curvature_accel = 0;
  Twist2d previous = out.state.velocity;
  for (std::size_t tick = 0; tick < std::max(ticks, std::size_t{1}); ++tick) {
    const auto before = out.state.velocity;
    detail::integrate_constant(out.state.pose, before, h);
    length += std::hypot(before.vx, before.vy) * h;
    largest_speed_jump =
      std::max(largest_speed_jump, std::hypot(before.vx - previous.vx, before.vy - previous.vy));
    curvature_accel =
      std::max(curvature_accel, std::hypot(before.vx, before.vy) * std::abs(before.wz));
    previous = before;
    const double now = start.stamp_s + (tick + 1) * h;
    const double wheel_delta = config_.max_wheel_accel_mps2 / config_.wheel_radius_m * h;
    const double steer_delta = config_.max_steer_rate_radps * h;
    bool drive = false;
    if (m.phase == TransitionPhase::Stable) {
      if (velocity.vx == 0 && velocity.vy == 0 && velocity.wz == 0) {
        approach(m.commanded_wheel_radps, std::array<double, 4>{}, wheel_delta);
        m.limited_velocity = {};
        drive = true;  // ordinary zero holds commanded steering
      } else {
        const auto & v = m.limited_velocity;
        const double dx = velocity.vx - v.vx, dy = velocity.vy - v.vy, dw = velocity.wz - v.wz;
        const double linear = std::hypot(dx, dy), angular = std::abs(dw);
        const double scale = std::min(
          {1.0, linear ? config_.max_linear_accel_mps2 * h / linear : 1.0,
           angular ? config_.max_angular_accel_radps2 * h / angular : 1.0});
        const Control next{v.vx + scale * dx, v.vy + scale * dy, v.wz + scale * dw};
        const auto w = targets(next);
        if (!w.valid) {
          out.valid = false;
          return out;
        }
        bool ready = true;
        for (std::size_t i = 0; i < 4; ++i)
          ready = ready && std::abs(w.angles[i] - out.state.steering_angles[i]) <=
                             config_.drive_steering_limit_rad;
        if (ready) {
          approach(m.commanded_wheel_radps, w.speeds, wheel_delta);
          approach(m.commanded_angles, w.angles, steer_delta);
          m.limited_velocity = {next.vx, next.vy, next.wz};
          drive = true;
        } else {
          m.alignment = targets(velocity).angles;
          m.phase = TransitionPhase::Braking;
          m.transition_start_s = now;
          m.aligned_since_s = -1;
        }
      }
    }
    if (!drive) {
      out.aligning = true;
      if (now - m.transition_start_s > config_.confirmation_timeout_s) {
        out.valid = false;
        return out;
      }
      approach(m.commanded_wheel_radps, std::array<double, 4>{}, wheel_delta);
      m.limited_velocity = {};
      const bool stopped = std::all_of(
        out.state.wheel_speeds.begin(), out.state.wheel_speeds.end(),
        [&](double v) { return std::abs(v) <= config_.stopped_wheel_speed_mps; });
      const bool commanded_zero = std::all_of(
        m.commanded_wheel_radps.begin(), m.commanded_wheel_radps.end(),
        [](double v) { return v == 0; });
      if (!stopped || !commanded_zero) {
        m.phase = TransitionPhase::Braking;
        m.aligned_since_s = -1;
      } else {
        m.phase = TransitionPhase::Aligning;
        approach(m.commanded_angles, m.alignment, steer_delta);
        if (detail::steering_aligned(out.state.steering_angles, m.alignment, config_)) {
          if (m.aligned_since_s < 0) m.aligned_since_s = now;
          if (now - m.aligned_since_s >= config_.alignment_min_s) m.phase = TransitionPhase::Stable;
        } else
          m.aligned_since_s = -1;
      }
    }
    out.state.steering_angles = m.commanded_angles;
    for (std::size_t i = 0; i < 4; ++i)
      out.state.wheel_speeds[i] = m.commanded_wheel_radps[i] * config_.wheel_radius_m;
    out.state.velocity = kinematics_.forward(out.state.wheel_speeds, m.commanded_angles);
  }
  out.steering_targets = m.commanded_angles;
  out.wheel_speed_targets = out.state.wheel_speeds;
  out.state.stamp_s += dt;
  out.state.time_in_mode_s += dt;
  // Discrete velocity jumps represented by a bounded linear ramp plus its
  // sample-and-hold discrepancy. Also cover curved motion and reversal.
  out.sweep_margin_m = std::min(
    length / 2, (largest_speed_jump / h + curvature_accel) * dt * dt / 8 + largest_speed_jump * dt);
  return out;
}
}  // namespace swerve_mppi
