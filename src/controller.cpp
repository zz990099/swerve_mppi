#include "swerve_mppi/controller.hpp"

#include "validation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_mppi {
Controller::Controller(const Config &config)
    : config_(config), model_(config), optimizer_(config), scheduler_(config),
      mode_manager_(config), path_manager_(config), goal_manager_(config) {}

Output Controller::compute(const ControllerInput &input) {
  Output stop;
  stop.requested_mode = input.vehicle.actual_mode;
  stop.phase = mode_manager_.phase();
  if (!detail::valid_input(input, config_) ||
      (last_stamp_s_ >= 0.0 && input.vehicle.stamp_s <= last_stamp_s_))
    return stop;
  stop.steering_targets = input.vehicle.steering_angles;
  last_stamp_s_ = input.vehicle.stamp_s;
  PathReference path;
  try {
    path = path_manager_.update(input);
  } catch (const std::invalid_argument &) {
    return stop;
  }
  if (path.changed) {
    optimizer_.reset();
    goal_manager_.reset();
    alignment_control_.reset();
  }
  ControllerInput prepared = input;
  prepared.reference_path = path.local_path;
  double limit = std::max(config_.max_vx_mps, config_.max_crab_speed_mps);
  const double distance =
      std::hypot(path.goal.x - input.vehicle.pose.x, path.goal.y - input.vehicle.pose.y);
  if (path.remaining_m < config_.goal_slowdown_distance_m)
    limit = std::min({limit, config_.goal_translation_gain * distance,
                      std::sqrt(2 * config_.max_linear_decel_mps2 *
                                std::max(0.0, distance - config_.goal_position_tolerance_m / 2))});
  prepared.tracking =
      TrackingContext{path.goal, path.remaining_m, limit, path.terminal, input.heading_policy};
  const auto goal = goal_manager_.update(input.vehicle, path,
                                         mode_manager_.active() || alignment_control_.has_value());
  Output out;
  if (mode_manager_.active()) {
    out = mode_manager_.update(input.vehicle);
    if (out.action == Action::Hold && out.phase == TransitionPhase::Stable)
      alignment_start_s_ = input.vehicle.stamp_s;
  } else if (!input.vehicle.mode_confirmed || input.vehicle.mode_fault) {
    out = stop;
  } else if (goal.complete || goal.position_acquired ||
             (path.remaining_m < config_.goal_docking_distance_m &&
              distance < config_.goal_docking_distance_m)) {
    alignment_control_.reset();
    optimizer_.reset();
    out = compute_goal(prepared, goal);
  } else if (path.corner_target && std::hypot(path.local_path.back().x - input.vehicle.pose.x,
                                              path.local_path.back().y - input.vehicle.pose.y) <
                                       config_.goal_docking_distance_m) {
    alignment_control_.reset();
    optimizer_.reset();
    auto corner = prepared;
    corner.tracking->goal = path.local_path.back();
    const double d = std::hypot(corner.tracking->goal.x - input.vehicle.pose.x,
                                corner.tracking->goal.y - input.vehicle.pose.y);
    corner.tracking->speed_limit_mps = std::min(limit, config_.goal_translation_gain * d);
    GoalState approach;
    out = compute_goal(corner, approach);
  } else if (alignment_control_) {
    out = continue_alignment(prepared);
  } else {
    out = compute_tracking(prepared);
  }
  out.navigation_status = out.action == Action::SafeStop ? NavigationStatus::Fault : goal.status;
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
  Output stop;
  stop.requested_mode = input.vehicle.actual_mode;
  stop.steering_targets = input.vehicle.steering_angles;
  const auto branches = scheduler_.make_branches(input.vehicle);
  std::vector<Solution> solutions;
  solutions.reserve(branches.size());
  for (const auto &branch : branches)
    solutions.push_back(optimizer_.optimize(input, branch));
  const auto &keep = solutions.front();
  const auto &best = solutions[scheduler_.select(solutions)];
  if (!std::isfinite(best.cost))
    return stop;
  if (best.branch.switches && best.branch.switch_step == 0) {
    auto out = request_mode(input, best.branch.mode, best.controls.front());
    out.selected_cost = best.cost;
    out.keep_cost = keep.cost;
    out.feasible_rollouts = best.feasible_rollouts;
    return out;
  }
  const auto preview =
      model_.step(input.vehicle, model_.project(best.controls.front(), input.vehicle.actual_mode),
                  config_.dt_s);
  if (!preview.valid)
    return stop;
  Output out;
  out.requested_mode = input.vehicle.actual_mode;
  out.selected_cost = best.cost;
  out.keep_cost = keep.cost;
  out.feasible_rollouts = best.feasible_rollouts;
  out.steering_targets = preview.steering_targets;
  if (preview.aligning) {
    alignment_control_ = model_.project(best.controls.front(), input.vehicle.actual_mode);
    alignment_mode_ = input.vehicle.actual_mode;
    alignment_start_s_ = input.vehicle.stamp_s;
    optimizer_.reset();
    out = continue_alignment(input);
    out.selected_cost = best.cost;
    out.keep_cost = keep.cost;
    out.feasible_rollouts = best.feasible_rollouts;
  } else {
    out.action = Action::Drive;
    out.body_command = preview.state.velocity;
    out.wheel_speed_targets = preview.wheel_speed_targets;
    optimizer_.accept(best, input.vehicle.actual_mode);
  }
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
  auto controls = std::vector<Control>(config_.horizon_steps, control);
  const auto trace =
      RolloutEngine(config_).generate(input.vehicle, {target_mode, 0, switching}, controls);
  if (!std::isfinite(CriticManager(config_).score(input, trace))) {
    out.action = Action::SafeStop;
    return out;
  }
  if (switching)
    return request_mode(input, target_mode, control);
  const auto step = model_.step(input.vehicle, control, config_.dt_s);
  if (!step.valid) {
    out.action = Action::SafeStop;
    return out;
  }
  out.steering_targets = step.steering_targets;
  if (step.aligning)
    out.phase = out.action == Action::Hold ? TransitionPhase::Aligning : TransitionPhase::Braking;
  if (!step.aligning) {
    out.action = Action::Drive;
    out.body_command = step.state.velocity;
    out.wheel_speed_targets = step.wheel_speed_targets;
  }
  return out;
}
Output Controller::continue_alignment(const ControllerInput &input) {
  Output out;
  out.requested_mode = input.vehicle.actual_mode;
  out.steering_targets = input.vehicle.steering_angles;
  if (input.vehicle.actual_mode != alignment_mode_ ||
      input.vehicle.stamp_s - alignment_start_s_ > config_.confirmation_timeout_s) {
    out.phase = TransitionPhase::Fault;
    return out;
  }
  // Recheck the committed intent against fresh obstacles while the optimizer
  // is paused. This deliberately conservative continuation may stop early.
  const auto continuation = RolloutEngine(config_).generate(
      input.vehicle, {alignment_mode_, 0, false},
      std::vector<Control>(config_.horizon_steps, *alignment_control_));
  if (!std::isfinite(CriticManager(config_).score(input, continuation)))
    return out;
  const auto preview = model_.step(input.vehicle, *alignment_control_, config_.dt_s);
  if (!preview.valid)
    return out;
  out.steering_targets = preview.steering_targets;
  if (preview.aligning) {
    out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
    out.phase = out.action == Action::Hold ? TransitionPhase::Aligning : TransitionPhase::Braking;
  } else {
    out.action = Action::Drive;
    out.body_command = preview.state.velocity;
    out.wheel_speed_targets = preview.wheel_speed_targets;
    alignment_control_.reset();
  }
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
