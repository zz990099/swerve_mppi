#include "swerve_mppi/controller.hpp"

#include "validation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_mppi {
Controller::Controller(const Config &config, std::shared_ptr<const TrajectoryValidator> validator)
    : validator_(validator ? std::move(validator) : std::make_shared<TrajectoryValidator>(config)),
      safety_rollout_(config), config_(config), model_(config), optimizer_(config, validator_),
      scheduler_(config), mode_manager_(config), path_manager_(config), goal_manager_(config) {
  validator_->require_compatible(config_);
}

Output Controller::compute(const ControllerInput &input) {
  Output stop;
  stop.requested_mode = input.vehicle.actual_mode;
  stop.phase = mode_manager_.phase();
  if (!detail::valid_input(input, config_)) {
    stop.failure_reason = FailureReason::InvalidInput;
    return stop;
  }
  if (last_stamp_s_ >= 0 && input.vehicle.stamp_s <= last_stamp_s_) {
    stop.failure_reason = FailureReason::NonmonotonicTime;
    return stop;
  }
  stop.steering_targets = input.vehicle.steering_angles;
  last_stamp_s_ = input.vehicle.stamp_s;
  PathReference path;
  try {
    path = path_manager_.update(input);
  } catch (const std::invalid_argument &) {
    stop.failure_reason = FailureReason::InvalidPath;
    return stop;
  }
  if (path.changed) {
    optimizer_.reset();
    goal_manager_.reset();
    alignment_control_.reset();
  }
  ControllerInput prepared;
  prepared.vehicle = input.vehicle;
  prepared.reference_path = path.local_path;
  prepared.obstacles = input.obstacles;
  prepared.path_id = input.path_id;
  prepared.heading_policy = input.heading_policy;
  double limit = std::max(config_.max_vx_mps, config_.max_crab_speed_mps);
  const double distance =
      std::hypot(path.target.x - input.vehicle.pose.x, path.target.y - input.vehicle.pose.y);
  if ((path.target_kind == PathTargetKind::Corner || path.goal_eligible) &&
      path.target_remaining_m < config_.goal_slowdown_distance_m)
    limit = std::min({limit, config_.goal_translation_gain * distance,
                      std::sqrt(2 * config_.max_linear_decel_mps2 *
                                std::max(0.0, distance - config_.goal_position_tolerance_m / 2))});
  prepared.tracking = TrackingContext{path.target, path.remaining_m, limit, path.goal_eligible,
                                      input.heading_policy};
  const auto goal = goal_manager_.update(input.vehicle, path,
                                         mode_manager_.active() || alignment_control_.has_value());
  Output out;
  if (mode_manager_.active()) {
    out = mode_manager_.update(input.vehicle);
    out.control_policy = ControlPolicy::ModeTransition;
    if (out.action == Action::SafeStop)
      out.failure_reason = FailureReason::TransitionFault;
    if (out.action == Action::Hold && out.phase == TransitionPhase::Stable)
      alignment_start_s_ = input.vehicle.stamp_s;
  } else if (!input.vehicle.mode_confirmed || input.vehicle.mode_fault) {
    out = stop;
    out.failure_reason = FailureReason::FeedbackFault;
  } else if (alignment_control_) {
    out = continue_alignment(prepared);
    out.control_policy = ControlPolicy::Alignment;
  } else if (goal.complete ||
             (path.goal_eligible &&
              (goal.position_acquired || (path.remaining_m < config_.goal_docking_distance_m &&
                                          distance < config_.goal_docking_distance_m)))) {
    optimizer_.reset();
    out = compute_goal(prepared, goal);
    out.control_policy = goal.complete ? ControlPolicy::Stopped : ControlPolicy::Capture;
  } else if (path.target_kind == PathTargetKind::Corner &&
             distance < config_.goal_docking_distance_m) {
    optimizer_.reset();
    GoalState approach;
    out = compute_goal(prepared, approach);
    out.control_policy = ControlPolicy::Capture;
  } else {
    out = compute_tracking(prepared);
    out.control_policy = ControlPolicy::Tracking;
  }
  // Every non-driving healthy action must admit a complete stop under the
  // current constraints, including terminal early returns and pending requests.
  if (out.action == Action::Brake || out.action == Action::Hold ||
      out.action == Action::RequestMode)
    out = check_stopping(prepared, std::move(out));
  if (out.action == Action::SafeStop)
    out.control_policy = ControlPolicy::Fault;
  out.navigation_status = out.action == Action::SafeStop ? NavigationStatus::Fault : goal.status;
  if (out.failure_reason == FailureReason::NoFeasiblePlan && out.action != Action::SafeStop) {
    out.control_policy = ControlPolicy::Blocked;
    out.navigation_status = NavigationStatus::Waiting;
  }
  out.goal_reached = goal.complete && out.action != Action::SafeStop;
  out.stalled = goal.stalled;
  out.path_progress_m = path.progress_m;
  out.remaining_path_m = path.remaining_m;
  out.cross_track_error_m = path.cross_track_error_m;
  out.goal_distance_m = goal.distance_m;
  out.goal_yaw_error_rad = goal.yaw_error_rad;
  return out;
}
Output Controller::compute_tracking(const ControllerInput &input) {
  const auto branches = scheduler_.make_branches(input.vehicle);
  PlanningStats stats;
  std::vector<Solution> solutions;
  solutions.reserve(branches.size());
  for (const auto &branch : branches) {
    solutions.push_back(optimizer_.optimize(input, branch));
    const auto &work = solutions.back().planning_stats;
    stats.branches += work.branches;
    stats.evaluated_rollouts += work.evaluated_rollouts;
    stats.feasible_rollouts += work.feasible_rollouts;
    stats.fallback_updates += work.fallback_updates;
  }
  const auto &keep = solutions.front();
  const auto &best = solutions[scheduler_.select(solutions)];
  if (!std::isfinite(best.cost)) {
    auto out = planning_stop(input);
    out.planning_stats = stats;
    return out;
  }
  if (best.branch.switches && best.branch.switch_step == 0) {
    auto out = request_mode(input, best.branch.mode, best.controls.front());
    out.planning_stats = stats;
    out.selected_cost = best.cost;
    out.keep_cost = keep.cost;
    out.feasible_rollouts = best.feasible_rollouts;
    return out;
  }
  auto out = apply_control(input, model_.project(best.controls.front(), input.vehicle.actual_mode));
  out.planning_stats = stats;
  out.selected_cost = best.cost;
  out.keep_cost = keep.cost;
  out.feasible_rollouts = best.feasible_rollouts;
  if (out.action == Action::Drive)
    optimizer_.accept(best, input.vehicle.actual_mode);
  return out;
}
Output Controller::request_mode(const ControllerInput &input, DriveMode mode,
                                const Control &intent) {
  Output stop;
  stop.requested_mode = input.vehicle.actual_mode;
  stop.steering_targets = input.vehicle.steering_angles;
  try {
    mode_manager_.begin(mode, intent, input.vehicle);
  } catch (const std::overflow_error &) {
    stop.failure_reason = FailureReason::TransitionFault;
    return stop;
  }
  optimizer_.reset();
  alignment_control_ = intent;
  alignment_mode_ = mode;
  alignment_start_s_ = input.vehicle.stamp_s;
  return mode_manager_.update(input.vehicle);
}
Output Controller::compute_goal(const ControllerInput &input, const GoalState &goal) {
  Output out;
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
  if (goal.complete)
    return out;
  Control control;
  DriveMode target_mode = DriveMode::Spin;
  if (goal.position_acquired) {
    if (std::abs(goal.yaw_error_rad) <= config_.goal_yaw_tolerance_rad)
      return out;
    const double error = std::abs(goal.yaw_error_rad);
    const double speed =
        std::min({config_.max_spin_radps, config_.goal_rotation_gain * error,
                  std::sqrt(2 * config_.max_angular_decel_radps2 *
                            std::max(0.0, error - config_.goal_yaw_tolerance_rad / 2))});
    control.wz = std::copysign(speed, goal.yaw_error_rad);
  } else {
    const auto &pose = input.vehicle.pose;
    const auto &target = input.tracking->goal;
    const double dx = target.x - pose.x, dy = target.y - pose.y;
    const double x = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
    const double y = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
    target_mode = input.vehicle.actual_mode == DriveMode::DualAckermann &&
                          std::abs(y) < config_.goal_position_tolerance_m / 2
                      ? DriveMode::DualAckermann
                      : DriveMode::Crab;
    control = {config_.goal_translation_gain * x,
               target_mode == DriveMode::Crab ? config_.goal_translation_gain * y : 0, 0};
    const double speed = std::hypot(control.vx, control.vy);
    const double limit = input.tracking->speed_limit_mps;
    if (speed > limit) {
      control.vx *= limit / speed;
      control.vy *= limit / speed;
    }
    control = model_.project(control, target_mode);
  }
  const bool switching = input.vehicle.actual_mode != target_mode;
  if (switching && (!is_stopped(input.vehicle, config_) ||
                    input.vehicle.time_in_mode_s < config_.minimum_mode_dwell_s))
    return out;
  if (switching && !safe_control(input, {target_mode, 0, true}, control))
    return planning_stop(input);
  if (switching)
    return request_mode(input, target_mode, control);
  return apply_control(input, control);
}
Output Controller::continue_alignment(const ControllerInput &input) {
  Output out;
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  if (input.vehicle.actual_mode != alignment_mode_ ||
      input.vehicle.stamp_s - alignment_start_s_ > config_.confirmation_timeout_s) {
    out.phase = TransitionPhase::Fault;
    out.failure_reason = FailureReason::TransitionFault;
    return out;
  }
  // Check the committed first Drive and its complete stopping continuation,
  // rather than extrapolating a capture command beyond its target for 2 seconds.
  if (!safe_control(input, {alignment_mode_, 0, false}, *alignment_control_))
    return planning_stop(input);
  return apply_control(input, *alignment_control_);
}
Output Controller::apply_control(const ControllerInput &input, Control control) {
  Output out;
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  // A zero intent is braking even when measured joints still imply motion.
  // Preserve the executor's braking semantics instead of declaring a Drive.
  if (std::hypot(control.vx, control.vy) < 1e-9 && std::abs(control.wz) < 1e-9) {
    alignment_control_.reset();
    optimizer_.reset();
    out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
    return out;
  }
  const auto preview = model_.step(input.vehicle, control, config_.dt_s);
  if (!preview.valid) {
    out.failure_reason = FailureReason::ModelFailure;
    return out;
  }
  out.steering_targets = preview.steering_targets;
  if (preview.aligning) {
    if (!alignment_control_) {
      alignment_control_ = control;
      alignment_mode_ = input.vehicle.actual_mode;
      alignment_start_s_ = input.vehicle.stamp_s;
      optimizer_.reset();
      return continue_alignment(input);
    }
    out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
    out.phase = out.action == Action::Hold ? TransitionPhase::Aligning : TransitionPhase::Braking;
  } else {
    // All policies enter Drive through this gate. Warm starts are accepted only
    // after the selected first Drive and its entire stopping tail pass validation.
    if (!safe_control(input, {input.vehicle.actual_mode, 0, false}, control))
      return planning_stop(input);
    out.action = Action::Drive;
    out.body_command = preview.state.velocity;
    out.wheel_speed_targets = preview.wheel_speed_targets;
    alignment_control_.reset();
  }
  return out;
}
bool Controller::safe_control(const ControllerInput &input, const Branch &branch,
                              const Control &control) {
  safety_rollout_.generate_continuation(input.vehicle, branch, control, safety_trace_);
  return validator_->check(input, safety_trace_) == TrajectoryStatus::Valid &&
         is_stopped(safety_trace_.final_state, config_);
}
Output Controller::planning_stop(const ControllerInput &input) {
  optimizer_.reset();
  alignment_control_.reset();
  Output out;
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
  out.failure_reason = FailureReason::NoFeasiblePlan;
  return out;
}
Output Controller::check_stopping(const ControllerInput &input, Output out) {
  safety_rollout_.generate_stop(input.vehicle, safety_trace_);
  if (validator_->check(input, safety_trace_) == TrajectoryStatus::Valid &&
      is_stopped(safety_trace_.final_state, config_))
    return out;
  optimizer_.reset();
  alignment_control_.reset();
  out.action = Action::SafeStop;
  out.failure_reason = FailureReason::UnsafeStoppingTrajectory;
  out.mode_request.reset();
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  out.wheel_speed_targets.fill(0.0);
  out.body_command = {};
  return out;
}
void Controller::reset() {
  optimizer_.reset();
  mode_manager_.reset();
  last_stamp_s_ = -1.0;
  alignment_control_.reset();
  path_manager_.reset();
  goal_manager_.reset();
}
} // namespace swerve_mppi
