#include "swerve_mppi/executor.hpp"

#include "drive_interpolation.hpp"
#include "time_comparison.hpp"
#include "validation.hpp"
#include <algorithm>
#include <stdexcept>

namespace swerve_mppi {
namespace {
bool same_request(const ModeRequest &a, const ModeRequest &b) {
  return a.id == b.id && a.mode == b.mode && a.steering_targets == b.steering_targets;
}
bool valid_entry(const ModeRequest &request, const Config &config) {
  const auto &angles = request.steering_targets;
  if (request.mode == DriveMode::Crab) {
    for (double angle : angles)
      if (std::abs(std::sin(angle - angles[0])) > 1e-7)
        return false;
    return true;
  }
  Control intent{0, 0, 1};
  if (request.mode == DriveMode::DualAckermann) {
    // The FL wheel determines the common Ackermann curvature. All other
    // modules must describe the same instantaneous centre of rotation.
    const double denominator =
        std::sin(angles[0]) * config.track_m / 2 + std::cos(angles[0]) * config.wheelbase_m / 2;
    if (std::abs(denominator) < 1e-9)
      return false;
    const double curvature = std::sin(angles[0]) / denominator;
    if (std::abs(curvature) > 1.0 / config.min_turn_radius_m + 1e-7)
      return false;
    intent = {1, 0, curvature};
  }
  const auto expected = Kinematics(config).inverse(intent, angles);
  for (std::size_t i = 0; i < angles.size(); ++i)
    if (std::abs(expected.angles[i] - angles[i]) > 1e-7)
      return false;
  return expected.valid;
}
} // namespace
ModeExecutor::ModeExecutor(const Config &config, DriveMode mode)
    : config_(config), actual_mode_(mode) {
  validate(config_);
  if (!detail::valid_mode(mode))
    throw std::invalid_argument("invalid executor initial mode");
}

ExecutionResult ModeExecutor::update(const Output &command, const VehicleState &measured) {
  ExecutionResult out;
  out.steering_targets = last_steering_;
  const bool valid = detail::valid_vehicle(measured, config_);
  if (valid) {
    last_steering_ = measured.steering_angles;
    out.steering_targets = last_steering_;
  }
  auto finish = [&]() {
    out.phase = phase_;
    out.feedback = {actual_mode_, phase_ == TransitionPhase::Stable,
                    phase_ == TransitionPhase::Fault, last_request_id_,
                    initialized_ && valid ? std::max(0.0, measured.stamp_s - confirmed_s_) : 0.0};
    return out;
  };
  auto fault = [&]() {
    phase_ = TransitionPhase::Fault;
    out.action = Action::SafeStop;
    out.wheel_speed_targets.fill(0.0);
    return finish();
  };
  if (!valid || !detail::valid_action(command.action) || measured.stamp_s <= last_stamp_s_ ||
      measured.mode_fault || phase_ == TransitionPhase::Fault || command.action == Action::SafeStop)
    return fault();
  last_stamp_s_ = measured.stamp_s;
  if (!initialized_) {
    confirmed_s_ = measured.stamp_s - measured.time_in_mode_s;
    initialized_ = true;
  }

  if (command.action == Action::RequestMode) {
    if (!command.mode_request)
      return fault();
    const auto &r = *command.mode_request;
    if (r.id == 0 || !detail::valid_mode(r.mode) || r.mode != command.requested_mode ||
        !detail::valid_steering(r.steering_targets, config_) || !valid_entry(r, config_) ||
        r.id < last_request_id_)
      return fault();
    if (r.id == last_request_id_) {
      if (!request_ || !same_request(r, *request_))
        return fault();
      // A retry after completion remains a stopped acknowledgement; no restart.
      if (phase_ == TransitionPhase::Stable) {
        if (!is_stopped(measured, config_) ||
            !detail::steering_aligned(measured.steering_angles, r.steering_targets, config_))
          return fault();
        out.action = Action::Hold;
        out.steering_targets = r.steering_targets;
        return finish();
      }
    } else {
      if (phase_ != TransitionPhase::Stable || r.mode == actual_mode_)
        return fault();
      request_ = r;
      last_request_id_ = r.id;
      start_s_ = measured.stamp_s;
      alignment_start_s_ = -1.0;
      phase_ = TransitionPhase::Braking;
    }
  } else if (command.mode_request) {
    return fault();
  }

  if (phase_ != TransitionPhase::Stable) {
    // Retries cannot renew the deadline; Drive/Hold/Brake cannot cancel a request.
    if (detail::deadline_exceeded(measured.stamp_s, start_s_, config_.confirmation_timeout_s))
      return fault();
    if (!is_stopped(measured, config_)) {
      phase_ = TransitionPhase::Braking;
      alignment_start_s_ = -1.0;
      out.action = Action::Brake;
      return finish();
    }
    if (phase_ == TransitionPhase::Braking) {
      phase_ = TransitionPhase::Aligning;
      alignment_start_s_ = measured.stamp_s;
    }
    out.action = Action::RequestMode;
    out.steering_targets = request_->steering_targets;
    if (detail::elapsed_at_least(measured.stamp_s, alignment_start_s_, config_.alignment_min_s) &&
        detail::steering_aligned(measured.steering_angles, out.steering_targets, config_)) {
      actual_mode_ = request_->mode;
      confirmed_s_ = measured.stamp_s;
      phase_ = TransitionPhase::Stable;
      out.action = Action::Hold;
    }
    return finish();
  }

  switch (command.action) {
  case Action::Drive: {
    if (!measured.mode_confirmed || measured.actual_mode != actual_mode_ ||
        command.requested_mode != actual_mode_ || !std::isfinite(command.body_command.vx) ||
        !std::isfinite(command.body_command.vy) || !std::isfinite(command.body_command.wz) ||
        !detail::valid_steering(command.steering_targets, config_))
      return fault();
    for (std::size_t i = 0; i < 4; ++i) {
      const double delta = std::abs(command.steering_targets[i] - measured.steering_angles[i]);
      if (delta > config_.drive_steering_limit_rad + 1e-9 ||
          delta > config_.max_steer_rate_radps * config_.dt_s + 1e-9)
        return fault();
    }
    for (double speed : command.wheel_speed_targets)
      if (!std::isfinite(speed) || std::abs(speed) > config_.max_wheel_speed_mps + 1e-9)
        return fault();
    const auto implied =
        Kinematics(config_).forward(command.wheel_speed_targets, command.steering_targets);
    VehicleState endpoint = measured;
    endpoint.velocity = implied;
    endpoint.steering_angles = command.steering_targets;
    endpoint.wheel_speeds = command.wheel_speed_targets;
    if (!detail::within_body_limits(implied, actual_mode_, config_) ||
        !detail::drive_interpolation_admissible(measured, endpoint, config_, config_.dt_s))
      return fault();
    if (std::abs(implied.vx - command.body_command.vx) > 1e-7 ||
        std::abs(implied.vy - command.body_command.vy) > 1e-7 ||
        std::abs(implied.wz - command.body_command.wz) > 1e-7)
      return fault();
    if (Kinematics(config_).max_module_residual(command.wheel_speed_targets,
                                                command.steering_targets, implied) >
        config_.drive_kinematic_tolerance_mps + 1e-9)
      return fault();
    // A bounded joint interpolation need not lie exactly on the ideal mode
    // manifold. Check the predicted joint pair, not the old measured angles.
    const auto projected =
        DriveModel(config_).project({implied.vx, implied.vy, implied.wz}, actual_mode_);
    double maximum_speed = 0.0;
    for (double speed : command.wheel_speed_targets)
      maximum_speed = std::max(maximum_speed, std::abs(speed));
    const double linear_error = maximum_speed * config_.drive_steering_limit_rad + 1e-7;
    const double radius = std::hypot(config_.wheelbase_m / 2, config_.track_m / 2);
    if (std::hypot(implied.vx - projected.vx, implied.vy - projected.vy) > linear_error ||
        std::abs(implied.wz - projected.wz) > linear_error / radius)
      return fault();
    out.action = Action::Drive;
    out.steering_targets = command.steering_targets;
    out.wheel_speed_targets = command.wheel_speed_targets;
    break;
  }
  case Action::Hold:
    if (!detail::valid_steering(command.steering_targets, config_))
      return fault();
    out.action = is_stopped(measured, config_) ? Action::Hold : Action::Brake;
    if (out.action == Action::Hold)
      out.steering_targets = command.steering_targets;
    break;
  case Action::Brake:
    out.action = Action::Brake;
    break;
  default:
    return fault();
  }
  return finish();
}

void ModeExecutor::reset(const VehicleState &recovered) {
  if (!detail::valid_vehicle(recovered, config_) || !is_stopped(recovered, config_) ||
      !recovered.mode_confirmed || recovered.mode_fault)
    throw std::invalid_argument("executor recovery requires verified stopped state");
  actual_mode_ = recovered.actual_mode;
  last_request_id_ = std::max(last_request_id_, recovered.mode_request_id);
  request_.reset();
  phase_ = TransitionPhase::Stable;
  last_stamp_s_ = -1.0;
  confirmed_s_ = recovered.stamp_s - recovered.time_in_mode_s;
  initialized_ = true;
  last_steering_ = recovered.steering_angles;
}
} // namespace swerve_mppi
