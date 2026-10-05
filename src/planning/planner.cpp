#include "planning/detail/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "common/detail/time_comparison.hpp"
#include "model/detail/drive_interpolation.hpp"
#include "safety/detail/validation.hpp"
#include "swerve_mppi/feedback/feedback.hpp"

namespace swerve_mppi::detail
{
Planner::Planner(
  const Config & config, std::shared_ptr<const TrajectoryValidator> validator,
  PlanningBudget::Now now)
: validator_(validator ? std::move(validator) : std::make_shared<TrajectoryValidator>(config)),
  safety_rollout_(config),
  safety_actuation_(config),
  config_(config),
  now_(std::move(now)),
  model_(config),
  optimizer_(config, validator_),
  scheduler_(config),
  mode_manager_(config),
  path_manager_(config),
  goal_manager_(config)
{
  validator_->require_compatible(config_);
}

JointCommand Planner::compute(const ControllerInput & input)
{
  PlanningBudget budget(config_.dt_s * config_.compute_budget_ratio, now_);
  auto out = compute_impl(input, budget);
  // A slow bounded rollout or user critic can cross the cooperative deadline.
  // Never publish a late Drive/request, even if it was feasible before timeout.
  if (out.planning_stats.budget_exhausted || budget.expired()) {
    optimizer_.clear_warm_start();
    alignment_control_.reset();
    out.action = Action::SafeStop;
    out.failure_reason = FailureReason::ComputeTimeout;
    out.control_policy = ControlPolicy::Fault;
    out.navigation_status = NavigationStatus::Fault;
    out.goal_reached = false;
    out.mode_request.reset();
    if (detail::valid_vehicle(input.vehicle, config_)) {
      out.requested_mode = input.vehicle.actual_mode;
      out.steering_targets = input.vehicle.steering_angles;
    }
    out.wheel_speed_targets.fill(0);
    out.body_command = {};
    out.planning_stats.budget_exhausted = true;
  }
  return out;
}
JointCommand Planner::compute_impl(const ControllerInput & input, const PlanningBudget & budget)
{
  safety_reductions_ = 0;
  JointCommand stop;
  stop.requested_mode = input.vehicle.actual_mode;
  stop.phase = mode_manager_.phase();
  if (
    input.reference_path.size() > config_.max_path_points ||
    input.obstacles.size() > config_.max_obstacles) {
    stop.failure_reason = FailureReason::WorkloadExceeded;
    return stop;
  }
  if (
    detail::valid_vehicle(input.vehicle, config_) &&
    check_model_feedback(input.vehicle, config_).status == FeedbackStatus::Inconsistent) {
    stop.steering_targets = input.vehicle.steering_angles;
    stop.failure_reason = FailureReason::InconsistentFeedback;
    return stop;
  }
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
    optimizer_.clear_warm_start();
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
  if (
    (path.target_kind == PathTargetKind::Corner || path.goal_eligible) &&
    path.target_remaining_m < config_.goal_slowdown_distance_m) {
    limit = std::min(
      {limit, config_.goal_translation_gain * distance,
       std::sqrt(
         2 * config_.max_linear_decel_mps2 *
         std::max(0.0, distance - config_.goal_position_tolerance_m / 2))});
  }
  prepared.tracking =
    TrackingContext{path.target, path.remaining_m, limit, path.goal_eligible, input.heading_policy};
  GoalState goal;
  try {
    goal = goal_manager_.update(
      input.vehicle, path, mode_manager_.active() || alignment_control_.has_value());
  } catch (const std::invalid_argument &) {
    stop.failure_reason = FailureReason::InvalidInput;
    return stop;
  }
  // GoalOnly terminal translation cannot advance in Spin. A finite horizon
  // plus switch cost can otherwise prefer yaw-only motion indefinitely. The
  // capture path still checks dwell and full switch/stop trajectories.
  const bool spin_terminal_translation = path.goal_eligible &&
                                         input.heading_policy == PathHeadingPolicy::GoalOnly &&
                                         input.vehicle.actual_mode == DriveMode::Spin;
  JointCommand out;
  if (mode_manager_.active()) {
    out = mode_manager_.update(input.vehicle);
    out.control_policy = ControlPolicy::ModeTransition;
    if (out.action == Action::SafeStop) {
      out.failure_reason = FailureReason::TransitionFault;
    }
    if (out.action == Action::Hold && out.phase == TransitionPhase::Stable) {
      alignment_start_s_ = input.vehicle.stamp_s;
    }
  } else if (!input.vehicle.mode_confirmed || input.vehicle.mode_fault) {
    out = stop;
    out.failure_reason = FailureReason::FeedbackFault;
  } else if (alignment_control_) {
    out = continue_alignment(prepared);
    out.control_policy = ControlPolicy::Alignment;
  } else if (
    goal.complete || spin_terminal_translation ||
    (path.goal_eligible &&
     (goal.position_acquired || (path.remaining_m < config_.goal_docking_distance_m &&
                                 distance < config_.goal_docking_distance_m)))) {
    optimizer_.clear_warm_start();
    out = compute_goal(prepared, goal);
    out.control_policy = goal.complete ? ControlPolicy::Stopped : ControlPolicy::Capture;
  } else if (
    path.target_kind == PathTargetKind::Corner && distance < config_.goal_docking_distance_m) {
    optimizer_.clear_warm_start();
    GoalState approach;
    out = compute_goal(prepared, approach);
    out.control_policy = ControlPolicy::Capture;
  } else {
    out = compute_tracking(prepared, budget);
    out.control_policy = ControlPolicy::Tracking;
  }
  // Check the actual non-driving interval, including stationary steering,
  // followed by its full stop. A brake-only trace cannot authorize alignment.
  if (
    out.action == Action::Brake || out.action == Action::Hold ||
    out.action == Action::RequestMode) {
    out = check_stopping(prepared, std::move(out));
  }
  if (out.action == Action::SafeStop) {
    out.control_policy = ControlPolicy::Fault;
  }
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
  out.safety_reductions = safety_reductions_;
  return out;
}
JointCommand Planner::compute_tracking(const ControllerInput & input, const PlanningBudget & budget)
{
  const auto branches = scheduler_.make_branches(input.vehicle);
  PlanningStats stats;
  std::vector<Solution> solutions;
  solutions.reserve(branches.size());
  if (!optimizer_.prepare(input)) {
    return planning_stop(input);
  }
  for (const auto & branch : branches) {
    solutions.push_back(optimizer_.optimize(input, branch, &budget, true));
    const auto & work = solutions.back().planning_stats;
    stats.branches += work.branches;
    stats.evaluated_rollouts += work.evaluated_rollouts;
    stats.feasible_rollouts += work.feasible_rollouts;
    stats.fallback_updates += work.fallback_updates;
    if (work.budget_exhausted || budget.expired()) {
      JointCommand out;  // Expired work cannot authorize another expensive
                         // stopping check.
      out.failure_reason = FailureReason::ComputeTimeout;
      out.planning_stats = stats;
      out.planning_stats.budget_exhausted = true;
      return out;
    }
  }
  const auto & keep = solutions.front();
  const auto & best = solutions[scheduler_.select(solutions)];
  if (!std::isfinite(best.cost)) {
    auto out = planning_stop(input);
    out.planning_stats = stats;
    return out;
  }
  if (best.branch.switches && best.branch.switch_step == 0) {
    const auto reduced = safe_reduction(input, best.branch, best.controls.front());
    if (!reduced) {
      auto out = std::isfinite(keep.cost) ? apply_control(input, keep.controls.front())
                                          : planning_stop(input);
      out.planning_stats = stats;
      out.selected_cost = out.keep_cost = keep.cost;
      out.feasible_rollouts = keep.feasible_rollouts;
      return out;
    }
    auto out = request_mode(input, best.branch.mode, *reduced);
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
  if (out.action == Action::Drive && safety_reductions_ == 0) {
    optimizer_.accept(best, input.vehicle.actual_mode);
  }
  return out;
}
JointCommand Planner::request_mode(
  const ControllerInput & input, DriveMode mode, const Control & intent)
{
  JointCommand stop;
  stop.requested_mode = input.vehicle.actual_mode;
  stop.steering_targets = input.vehicle.steering_angles;
  try {
    mode_manager_.begin(mode, intent, input.vehicle);
  } catch (const std::overflow_error &) {
    stop.failure_reason = FailureReason::TransitionFault;
    return stop;
  }
  optimizer_.clear_warm_start();
  alignment_control_ = intent;
  alignment_mode_ = mode;
  alignment_start_s_ = input.vehicle.stamp_s;
  return mode_manager_.update(input.vehicle);
}
JointCommand Planner::compute_goal(const ControllerInput & input, const GoalState & goal)
{
  JointCommand out;
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
  if (goal.complete) {
    return out;
  }
  Control control;
  DriveMode target_mode = DriveMode::Spin;
  if (goal.position_acquired) {
    if (std::abs(goal.yaw_error_rad) <= config_.goal_yaw_tolerance_rad) {
      return out;
    }
    const double error = std::abs(goal.yaw_error_rad);
    const double speed = std::min(
      {config_.max_spin_radps, config_.goal_rotation_gain * error,
       std::sqrt(
         2 * config_.max_angular_decel_radps2 *
         std::max(0.0, error - config_.goal_yaw_tolerance_rad / 2))});
    control.wz = std::copysign(speed, goal.yaw_error_rad);
  } else {
    const auto & pose = input.vehicle.pose;
    const auto & target = input.tracking->goal;
    const double dx = target.x - pose.x, dy = target.y - pose.y;
    const double heading = wrap_angle(pose.yaw);
    const double x = std::cos(heading) * dx + std::sin(heading) * dy;
    const double y = -std::sin(heading) * dx + std::cos(heading) * dy;
    target_mode = input.vehicle.actual_mode == DriveMode::DualAckermann &&
                      std::abs(y) < config_.goal_position_tolerance_m / 2
                    ? DriveMode::DualAckermann
                    : DriveMode::Crab;
    control = {
      config_.goal_translation_gain * x,
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
  if (
    switching &&
    (!is_stopped(input.vehicle, config_) ||
     !detail::elapsed_at_least(input.vehicle.time_in_mode_s, 0, config_.minimum_mode_dwell_s))) {
    return out;
  }
  if (switching) {
    const auto reduced = safe_reduction(input, {target_mode, 0, true}, control);
    return reduced ? request_mode(input, target_mode, *reduced) : planning_stop(input);
  }
  return apply_control(input, control);
}
JointCommand Planner::continue_alignment(const ControllerInput & input)
{
  JointCommand out;
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  if (
    input.vehicle.actual_mode != alignment_mode_ ||
    detail::deadline_exceeded(
      input.vehicle.stamp_s, alignment_start_s_, config_.confirmation_timeout_s)) {
    out.phase = TransitionPhase::Fault;
    out.failure_reason = FailureReason::TransitionFault;
    return out;
  }
  return apply_control(input, *alignment_control_);
}
JointCommand Planner::apply_control(const ControllerInput & input, Control control)
{
  JointCommand out;
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  // A zero intent is braking even when measured joints still imply motion.
  // Preserve the executor's braking semantics instead of declaring a Drive.
  if (std::hypot(control.vx, control.vy) < 1e-9 && std::abs(control.wz) < 1e-9) {
    alignment_control_.reset();
    optimizer_.clear_warm_start();
    out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
    return out;
  }
  const auto measured =
    Kinematics(config_).forward(input.vehicle.wheel_speeds, input.vehicle.steering_angles);
  if (!detail::within_body_limits(measured, input.vehicle.actual_mode, config_)) {
    return planning_stop(input);
  }
  const auto reduced = safe_reduction(input, {input.vehicle.actual_mode, 0, false}, control);
  if (!reduced) {
    return planning_stop(input);
  }
  control = *reduced;
  if (alignment_control_) {
    alignment_control_ = control;
  }
  const auto preview = model_.step(input.vehicle, control, config_.dt_s);
  if (!preview.valid) {
    out.failure_reason = FailureReason::ModelFailure;
    return out;
  }
  out.velocity_intent = {control.vx, control.vy, control.wz};
  out.steering_targets = preview.steering_targets;
  if (preview.aligning) {
    if (!alignment_control_) {
      alignment_control_ = control;
      alignment_mode_ = input.vehicle.actual_mode;
      alignment_start_s_ = input.vehicle.stamp_s;
      optimizer_.clear_warm_start();
      return continue_alignment(input);
    }
    out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
    out.phase = out.action == Action::Hold ? TransitionPhase::Aligning : TransitionPhase::Braking;
  } else {
    // All policies enter Drive through this gate. Warm starts are accepted only
    // after the selected first Drive and its entire stopping tail pass
    // validation.
    if (!detail::within_body_limits(preview.state.velocity, input.vehicle.actual_mode, config_)) {
      return planning_stop(input);
    }
    out.action = Action::Drive;
    out.body_command = preview.state.velocity;
    out.wheel_speed_targets = preview.wheel_speed_targets;
    alignment_control_.reset();
  }
  return out;
}
std::optional<Control> Planner::safe_reduction(
  const ControllerInput & input, const Branch & branch, Control control)
{
  if (safe_control(input, branch, control)) {
    return control;
  }
  for (std::size_t attempt = 0; attempt < config_.safety_reduction_attempts; ++attempt) {
    control = {control.vx * .5, control.vy * .5, control.wz * .5};
    if (std::hypot(control.vx, control.vy) < 1e-9 && std::abs(control.wz) < 1e-9) {
      break;
    }
    ++safety_reductions_;
    if (safe_control(input, branch, control)) {
      optimizer_.clear_warm_start();
      return control;
    }
  }
  return std::nullopt;
}
bool Planner::safe_control(
  const ControllerInput & input, const Branch & branch, const Control & control)
{
  safety_rollout_.generate_continuation(input.vehicle, branch, control, safety_trace_);
  return validator_->check(input, safety_trace_) == TrajectoryStatus::Valid &&
         is_stopped(safety_trace_.final_state, config_);
}
JointCommand Planner::planning_stop(const ControllerInput & input)
{
  optimizer_.clear_warm_start();
  alignment_control_.reset();
  JointCommand out;
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
  out.failure_reason = FailureReason::NoFeasiblePlan;
  return out;
}
JointCommand Planner::check_stopping(const ControllerInput & input, JointCommand out)
{
  const auto plan =
    safety_actuation_.plan_stopping(input.vehicle, out.action, out.steering_targets);
  if (plan) {
    safety_rollout_.generate_execution(*plan, safety_trace_);
    if (
      validator_->check(input, safety_trace_) == TrajectoryStatus::Valid &&
      is_stopped(safety_trace_.final_state, config_)) {
      return out;
    }
  }
  optimizer_.clear_warm_start();
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
void Planner::reset()
{
  optimizer_.reset();
  mode_manager_.reset();
  last_stamp_s_ = -1.0;
  alignment_control_.reset();
  path_manager_.reset();
  goal_manager_.reset();
}
}  // namespace swerve_mppi::detail
