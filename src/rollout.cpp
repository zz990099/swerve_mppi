#include "swerve_mppi/rollout.hpp"
#include "motion_profile.hpp"
#include "time_comparison.hpp"
#include "validation.hpp"
#include <algorithm>
#include <cmath>

namespace swerve_mppi {
RolloutEngine::RolloutEngine(const Config &config)
    : config_(config), model_(config), transition_(config) {}
void RolloutEngine::generate_stop(const VehicleState &initial, Trajectory &out) const {
  stopping_rollout(initial, {initial.actual_mode, 0, false}, nullptr, out);
}
void RolloutEngine::generate_execution(const ActuationPlan &plan, Trajectory &out) const {
  out.valid = false;
  out.poses.clear();
  out.sweep_margins_m.clear();
  out.controls.clear();
  out.active_controls.clear();
  out.position_error_m = 0;
  out.branch = {plan.start().actual_mode, 0, false};
  out.final_state = plan.start();
  if (!plan.compatible_with(config_) || !detail::valid_vehicle(plan.start(), config_) ||
      !detail::valid_vehicle(plan.endpoint().state, config_))
    return;
  out.poses.push_back(plan.start().pose);
  detail::append_motion(plan.endpoint(), &out.poses, &out.sweep_margins_m, out.position_error_m);
  out.final_state = plan.endpoint().state;
  out.controls.push_back({out.final_state.velocity.vx, out.final_state.velocity.vy,
                          out.final_state.velocity.wz});
  out.active_controls.push_back(plan.action() == Action::Drive);
  std::size_t steps = 1;
  auto at_rest = [&]() {
    return out.final_state.velocity.vx == 0 && out.final_state.velocity.vy == 0 &&
           out.final_state.velocity.wz == 0 &&
           std::all_of(out.final_state.wheel_speeds.begin(), out.final_state.wheel_speeds.end(),
                       [](double speed) { return speed == 0; });
  };
  while (!at_rest()) {
    if (steps >= config_.stopping_horizon_steps)
      return;
    const auto next = model_.step(out.final_state, {}, config_.dt_s);
    if (!next.valid)
      return;
    out.final_state = next.state;
    detail::append_motion(next, &out.poses, &out.sweep_margins_m, out.position_error_m);
    out.controls.push_back({});
    out.active_controls.push_back(false);
    ++steps;
  }
  out.valid = true;
}
void RolloutEngine::generate_continuation(const VehicleState &initial, const Branch &branch,
                                          const Control &first_control, Trajectory &out) const {
  stopping_rollout(initial, branch, &first_control, out);
}
void RolloutEngine::stopping_rollout(const VehicleState &initial, const Branch &branch,
                                     const Control *first_control, Trajectory &out) const {
  out.valid = false;
  out.poses.clear();
  out.sweep_margins_m.clear();
  out.position_error_m = 0;
  out.controls.clear();
  out.active_controls.clear();
  out.branch = branch;
  out.final_state = initial;
  if (!detail::valid_vehicle(initial, config_) ||
      (first_control &&
       (!initial.mode_confirmed || initial.mode_fault || !std::isfinite(first_control->vx) ||
        !std::isfinite(first_control->vy) || !std::isfinite(first_control->wz))) ||
      (branch.switches &&
       (!first_control || branch.switch_step != 0 || branch.mode == initial.actual_mode ||
        !detail::elapsed_at_least(initial.time_in_mode_s, 0, config_.minimum_mode_dwell_s))))
    return;
  out.poses.push_back(initial.pose);
  std::size_t steps = 0;
  if (branch.switches &&
      transition_.rollout(out.final_state, branch.mode, steps, config_.stopping_horizon_steps,
                          &out.poses, *first_control, &out.sweep_margins_m,
                          &out.position_error_m) < 0)
    return;
  out.controls.resize(steps);
  out.active_controls.resize(steps, false);
  const std::size_t alignment_begin = steps;
  bool pending = first_control != nullptr;
  // Include residual movement below the executor's stopped thresholds. Those
  // thresholds permit handover, but do not certify zero remaining displacement.
  auto at_rest = [&]() {
    return out.final_state.velocity.vx == 0 && out.final_state.velocity.vy == 0 &&
           out.final_state.velocity.wz == 0 &&
           std::all_of(out.final_state.wheel_speeds.begin(), out.final_state.wheel_speeds.end(),
                       [](double speed) { return speed == 0; });
  };
  while (pending || !at_rest()) {
    if (steps >= config_.stopping_horizon_steps ||
        (pending && detail::duration_exceeded((steps - alignment_begin) * config_.dt_s,
                                              config_.confirmation_timeout_s)))
      return;
    const Control control =
        pending ? model_.project(*first_control, out.final_state.actual_mode) : Control{};
    const auto next = model_.step(out.final_state, control, config_.dt_s);
    if (!next.valid)
      return;
    out.final_state = next.state;
    detail::append_motion(next, &out.poses, &out.sweep_margins_m, out.position_error_m);
    out.controls.push_back(control);
    out.active_controls.push_back(first_control && steps == alignment_begin);
    if (!next.aligning)
      pending = false;
    ++steps;
  }
  out.valid = true;
}
Trajectory RolloutEngine::generate(const VehicleState &initial, const Branch &branch,
                                   const std::vector<Control> &controls) const {
  Trajectory out;
  generate(initial, branch, controls, out);
  return out;
}
void RolloutEngine::generate(const VehicleState &initial, const Branch &branch,
                             const std::vector<Control> &controls, Trajectory &out) const {
  out.valid = false;
  out.poses.clear();
  out.sweep_margins_m.clear();
  out.position_error_m = 0;
  out.controls.clear();
  out.active_controls.clear();
  out.branch = branch;
  out.final_state = initial;
  if (!detail::valid_vehicle(initial, config_) || !initial.mode_confirmed || initial.mode_fault ||
      controls.size() != config_.horizon_steps ||
      (branch.switches &&
       (branch.mode == initial.actual_mode || branch.switch_step >= config_.horizon_steps ||
        !detail::elapsed_at_least(initial.time_in_mode_s + branch.switch_step * config_.dt_s, 0,
                                  config_.minimum_mode_dwell_s))))
    return;
  out.poses.reserve(config_.horizon_steps + 1);
  out.poses.push_back(initial.pose);
  out.controls.resize(controls.size());
  out.active_controls.assign(controls.size(), false);
  std::size_t step = 0;
  bool switched = false;
  std::optional<Control> alignment;
  std::size_t alignment_begin = 0;
  while (step < config_.horizon_steps) {
    if (branch.switches && !switched && step == branch.switch_step) {
      if (alignment)
        return;
      if (transition_.rollout(out.final_state, branch.mode, step, config_.horizon_steps, &out.poses,
                              controls[branch.switch_step], &out.sweep_margins_m,
                              &out.position_error_m) < 0.0)
        return;
      switched = true;
      // The switch-entry control remains committed through the first Drive.
      // The proposal at the resume index is ignored, just as in Controller.
      alignment = model_.project(controls[branch.switch_step], branch.mode);
      alignment_begin = step;
      continue;
    }
    const DriveMode mode = switched ? branch.mode : initial.actual_mode;
    if (!std::isfinite(controls[step].vx) || !std::isfinite(controls[step].vy) ||
        !std::isfinite(controls[step].wz))
      return;
    const bool continuing_alignment = alignment.has_value();
    if (continuing_alignment && detail::duration_exceeded((step - alignment_begin) * config_.dt_s,
                                                          config_.confirmation_timeout_s))
      return;
    const Control control = alignment.value_or(model_.project(controls[step], mode));
    const auto next = model_.step(out.final_state, control, config_.dt_s);
    if (!next.valid)
      return;
    out.final_state = next.state;
    out.controls[step] = control;
    // The first control commits entry geometry. Following controls are ignored
    // until that same intent finishes alignment and emits its first Drive.
    out.active_controls[step] = !continuing_alignment;
    if (next.aligning && !alignment) {
      alignment = control;
      alignment_begin = step;
    } else if (!next.aligning) {
      alignment.reset();
    }
    detail::append_motion(next, &out.poses, &out.sweep_margins_m, out.position_error_m);
    ++step;
  }
  out.valid = out.poses.size() == config_.horizon_steps + 1;
}
} // namespace swerve_mppi
