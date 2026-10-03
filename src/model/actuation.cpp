#include "swerve_mppi/model/actuation.hpp"

#include <algorithm>
#include <cmath>

#include "model/detail/drive_interpolation.hpp"
#include "model/detail/stopping_motion.hpp"
#include "safety/detail/validation.hpp"
#include "swerve_mppi/feedback/feedback.hpp"

namespace swerve_mppi
{
namespace
{
std::array<double, 22> model_parameters(const Config & c)
{
  return {
    c.wheelbase_m,
    c.track_m,
    c.wheel_radius_m,
    c.max_wheel_speed_mps,
    c.max_wheel_accel_mps2,
    c.steering_limit_rad,
    c.max_steer_rate_radps,
    c.max_vx_mps,
    c.max_crab_speed_mps,
    c.max_spin_radps,
    c.max_yaw_rate_radps,
    c.min_turn_radius_m,
    c.max_linear_accel_mps2,
    c.max_linear_decel_mps2,
    c.max_angular_accel_radps2,
    c.max_angular_decel_radps2,
    c.drive_steering_limit_rad,
    c.drive_kinematic_tolerance_mps,
    c.stopped_linear_mps,
    c.stopped_angular_radps,
    c.stopped_wheel_speed_mps,
    c.dt_s};
}
bool valid_phase(TransitionPhase phase)
{
  return phase == TransitionPhase::Stable || phase == TransitionPhase::Braking ||
         phase == TransitionPhase::Aligning || phase == TransitionPhase::AwaitingConfirmation;
}
bool valid_endpoint(const StepResult & endpoint, const Config & config)
{
  return detail::valid_vehicle(endpoint.state, config) && std::isfinite(endpoint.sweep_margin_m) &&
         endpoint.sweep_margin_m >= 0 && std::isfinite(endpoint.integration_error_m) &&
         endpoint.integration_error_m >= 0;
}
}  // namespace
bool ActuationPlan::compatible_with(const Config & config) const
{
  return model_parameters_ == model_parameters(config);
}
std::optional<ActuatorTargets> ActuationPlan::sample(double elapsed_s) const
{
  if (!std::isfinite(elapsed_s) || elapsed_s < 0 || elapsed_s > duration_s_) {
    return std::nullopt;
  }
  ActuatorTargets targets;
  if (action_ == Action::Drive) {
    const double fraction = elapsed_s / duration_s_;
    for (std::size_t i = 0; i < 4; ++i) {
      targets.wheel_speeds[i] =
        start_.wheel_speeds[i] +
        fraction * (endpoint_.state.wheel_speeds[i] - start_.wheel_speeds[i]);
      targets.steering_angles[i] =
        start_.steering_angles[i] +
        fraction * (endpoint_.state.steering_angles[i] - start_.steering_angles[i]);
    }
  } else {
    const double scale =
      braking_duration_s_ > 0 ? std::max(0.0, 1 - elapsed_s / braking_duration_s_) : 0;
    const double alignment_time = may_align_ ? std::max(0.0, elapsed_s - braking_duration_s_) : 0;
    for (std::size_t i = 0; i < 4; ++i) {
      targets.wheel_speeds[i] = start_.wheel_speeds[i] * scale;
      targets.steering_angles[i] =
        start_.steering_angles[i] + std::clamp(
                                      endpoint_.steering_targets[i] - start_.steering_angles[i],
                                      -steering_rate_radps_ * alignment_time,
                                      steering_rate_radps_ * alignment_time);
    }
  }
  return targets;
}

ActuationModel::ActuationModel(const Config & config) : config_(config), kinematics_(config) {}
std::optional<ActuationPlan> ActuationModel::plan_stopping(
  const VehicleState & measured, Action action,
  const std::array<double, 4> & steering_targets) const
{
  if (
    check_model_feedback(measured, config_).status != FeedbackStatus::Valid ||
    measured.mode_fault || !detail::valid_steering(steering_targets, config_) ||
    (action != Action::Brake && action != Action::Hold && action != Action::RequestMode)) {
    return std::nullopt;
  }
  ActuationPlan out;
  out.start_ = measured;
  out.action_ = action;
  out.duration_s_ = config_.dt_s;
  out.steering_rate_radps_ = config_.max_steer_rate_radps;
  out.model_parameters_ = model_parameters(config_);
  const auto & targets = action == Action::Brake ? measured.steering_angles : steering_targets;
  out.braking_duration_s_ = detail::braking_duration(measured, config_);
  out.may_align_ = action != Action::Brake && is_stopped(measured, config_);
  out.endpoint_ = detail::stopping_step(measured, targets, config_, config_.dt_s);
  if (!valid_endpoint(out.endpoint_, config_)) {
    return std::nullopt;
  }
  return out;
}
std::optional<ActuationPlan> ActuationModel::plan(
  const VehicleState & measured, const ExecutionResult & execution) const
{
  if (
    check_model_feedback(measured, config_).status != FeedbackStatus::Valid ||
    measured.mode_fault || execution.feedback.fault || !valid_phase(execution.phase) ||
    execution.feedback.confirmed != (execution.phase == TransitionPhase::Stable) ||
    !detail::valid_mode(execution.feedback.actual_mode) ||
    !std::isfinite(execution.feedback.time_in_mode_s) || execution.feedback.time_in_mode_s < 0 ||
    !detail::valid_steering(execution.steering_targets, config_)) {
    return std::nullopt;
  }
  ActuationPlan out;
  out.start_ = measured;
  out.action_ = execution.action;
  out.duration_s_ = config_.dt_s;
  out.steering_rate_radps_ = config_.max_steer_rate_radps;
  out.model_parameters_ = model_parameters(config_);
  if (execution.action == Action::Drive) {
    if (
      !measured.mode_confirmed || measured.mode_fault || !execution.feedback.confirmed ||
      measured.actual_mode != execution.feedback.actual_mode ||
      !detail::drive_targets_admissible(
        measured, execution.steering_targets, execution.wheel_speed_targets, config_)) {
      return std::nullopt;
    }
    out.endpoint_.state = measured;
    out.endpoint_.state.wheel_speeds = execution.wheel_speed_targets;
    out.endpoint_.state.steering_angles = execution.steering_targets;
    out.endpoint_.state.velocity =
      kinematics_.forward(execution.wheel_speed_targets, execution.steering_targets);
    out.endpoint_.steering_targets = execution.steering_targets;
    out.endpoint_.wheel_speed_targets = execution.wheel_speed_targets;
    const auto before = kinematics_.forward(measured.wheel_speeds, measured.steering_angles);
    detail::integrate_drive(out.endpoint_, measured, before, config_, config_.dt_s);
    out.endpoint_.state.stamp_s += config_.dt_s;
  } else if (
    execution.action == Action::Brake || execution.action == Action::Hold ||
    execution.action == Action::RequestMode) {
    for (double speed : execution.wheel_speed_targets) {
      if (speed != 0) {
        return std::nullopt;
      }
    }
    const auto stopping = plan_stopping(measured, execution.action, execution.steering_targets);
    if (!stopping) {
      return std::nullopt;
    }
    out = *stopping;
  } else {
    return std::nullopt;
  }
  out.endpoint_.state.actual_mode = execution.feedback.actual_mode;
  out.endpoint_.state.mode_confirmed = execution.feedback.confirmed;
  out.endpoint_.state.mode_fault = execution.feedback.fault;
  out.endpoint_.state.mode_request_id = execution.feedback.request_id;
  out.endpoint_.state.time_in_mode_s = execution.feedback.time_in_mode_s + config_.dt_s;
  if (!valid_endpoint(out.endpoint_, config_)) {
    return std::nullopt;
  }
  return out;
}
}  // namespace swerve_mppi
