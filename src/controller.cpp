#include "swerve_mppi/controller.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_mppi {
namespace {
bool is_stopped(const Twist2d & v, const Config & c) {
  return std::hypot(v.vx, v.vy) <= c.stopped_linear_mps &&
         std::abs(v.wz) <= c.stopped_angular_radps;
}

bool valid_input(const ControllerInput & input) {
  if (input.reference_path.empty() || !std::isfinite(input.vehicle.stamp_s) ||
      !std::isfinite(input.vehicle.pose.x) ||
      !std::isfinite(input.vehicle.pose.y) ||
      !std::isfinite(input.vehicle.pose.yaw) ||
      !std::isfinite(input.vehicle.velocity.vx) ||
      !std::isfinite(input.vehicle.velocity.vy) ||
      !std::isfinite(input.vehicle.velocity.wz) ||
      !std::isfinite(input.vehicle.time_in_mode_s) ||
      input.vehicle.time_in_mode_s < 0.0) return false;
  for (double angle : input.vehicle.steering_angles) {
    if (!std::isfinite(angle)) return false;
  }
  for (double speed : input.vehicle.wheel_speeds) {
    if (!std::isfinite(speed)) return false;
  }
  for (const auto & waypoint : input.reference_path) {
    if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) ||
        !std::isfinite(waypoint.yaw)) return false;
  }
  for (const auto & obstacle : input.obstacles) {
    if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
        !std::isfinite(obstacle.radius) || obstacle.radius < 0.0) return false;
  }
  return true;
}
}  // namespace

void ModeManager::begin(DriveMode target_mode, double now_s) {
  target_mode_ = target_mode;
  start_s_ = now_s;
  phase_ = TransitionPhase::Braking;
}

Output ModeManager::update(const VehicleState & observed) {
  Output out;
  out.phase = phase_;
  out.requested_mode = target_mode_;
  if (observed.mode_fault || phase_ == TransitionPhase::Fault ||
      observed.stamp_s - start_s_ > config_.confirmation_timeout_s) {
    phase_ = TransitionPhase::Fault;
    out.phase = phase_;
    out.action = Action::SafeStop;
    return out;
  }
  if (phase_ == TransitionPhase::Braking) {
    if (is_stopped(observed.velocity, config_)) {
      phase_ = TransitionPhase::AwaitingConfirmation;
      out.action = Action::RequestMode;
    } else {
      out.action = Action::Brake;
    }
  } else if (phase_ == TransitionPhase::AwaitingConfirmation) {
    if (observed.actual_mode == target_mode_ && observed.mode_confirmed &&
        is_stopped(observed.velocity, config_)) {
      phase_ = TransitionPhase::Stable;
      out.action = Action::Hold;
    } else {
      out.action = Action::RequestMode;
    }
  } else {
    out.action = Action::Hold;
  }
  out.phase = phase_;
  return out;
}

void ModeManager::reset() { phase_ = TransitionPhase::Stable; }

Controller::Controller(const Config & config)
    : config_(config), model_(config_), optimizer_(config_),
      mode_manager_(config_) {
  validate(config_);
}

Output Controller::compute(const ControllerInput & input) {
  Output stop;
  stop.requested_mode = input.vehicle.actual_mode;
  stop.phase = mode_manager_.phase();
  if (!valid_input(input) ||
      (last_stamp_s_ >= 0.0 && input.vehicle.stamp_s <= last_stamp_s_)) {
    return stop;
  }
  last_stamp_s_ = input.vehicle.stamp_s;
  if (mode_manager_.active()) {
    Output out = mode_manager_.update(input.vehicle);
    if (out.action == Action::RequestMode) {
      out.steering_targets = model_.steering_for_mode(out.requested_mode);
    }
    return out;
  }
  if (!input.vehicle.mode_confirmed || input.vehicle.mode_fault) return stop;

  const auto branches = optimizer_.make_branches(input.vehicle);
  Solution keep = optimizer_.optimize(input, branches.front());
  Solution best = keep;
  for (std::size_t i = 1; i < branches.size(); ++i) {
    Solution candidate = optimizer_.optimize(input, branches[i]);
    if (candidate.cost < best.cost) best = std::move(candidate);
  }
  if (!std::isfinite(best.cost)) return stop;
  // A future switch remains a plan, never a premature mode request.
  if (best.branch.switches && best.branch.switch_step == 0 &&
      best.cost + config_.switch_hysteresis < keep.cost) {
    mode_manager_.begin(best.branch.mode, input.vehicle.stamp_s);
    Output out = mode_manager_.update(input.vehicle);
    if (out.action == Action::RequestMode) {
      out.steering_targets = model_.steering_for_mode(out.requested_mode);
    }
    out.selected_cost = best.cost;
    out.keep_cost = keep.cost;
    out.feasible_rollouts = best.feasible_rollouts;
    return out;
  }

  // If a switch is beneficial later in the horizon, execute only the current
  // mode's first step. Mode changes require a new decision with fresh feedback.
  const Control first = model_.project(best.controls.front(),
                                        input.vehicle.actual_mode);
  const StepResult preview = model_.step(input.vehicle, first, config_.dt_s);
  if (!preview.valid) return stop;
  Output out;
  out.requested_mode = input.vehicle.actual_mode;
  out.phase = TransitionPhase::Stable;
  out.selected_cost = best.cost;
  out.keep_cost = keep.cost;
  out.feasible_rollouts = best.feasible_rollouts;
  out.steering_targets = preview.steering_targets;
  bool aligning = false;
  for (std::size_t i = 0; i < out.steering_targets.size(); ++i) {
    if (std::abs(angle_distance(out.steering_targets[i],
                                input.vehicle.steering_angles[i])) >
        config_.steering_tolerance_rad) aligning = true;
  }
  if (aligning) {
    out.action = is_stopped(input.vehicle.velocity, config_) ?
        Action::Hold : Action::Brake;
  } else {
    out.action = Action::Drive;
    out.body_command = preview.state.velocity;
    out.wheel_speed_targets = preview.wheel_speed_targets;
  }
  return out;
}

void Controller::reset() {
  optimizer_.reset();
  mode_manager_.reset();
  last_stamp_s_ = -1.0;
}

}  // namespace swerve_mppi
