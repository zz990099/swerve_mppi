#include "swerve_mppi/rollout.hpp"
#include "validation.hpp"
#include <cmath>

namespace swerve_mppi {
RolloutEngine::RolloutEngine(const Config &config)
    : config_(config), model_(config), transition_(config) {}
void RolloutEngine::generate_stop(const VehicleState &initial, Trajectory &out) const {
  out.valid = false;
  out.poses.clear();
  out.controls.clear();
  out.active_controls.clear();
  out.branch = {initial.actual_mode, 0, false};
  out.final_state = initial;
  if (!detail::valid_vehicle(initial, config_))
    return;
  out.poses.reserve(config_.horizon_steps + 1);
  out.poses.push_back(initial.pose);
  out.controls.assign(config_.horizon_steps, Control{});
  out.active_controls.assign(config_.horizon_steps, false);
  for (std::size_t step = 0; step < config_.horizon_steps; ++step) {
    const auto next = model_.step(out.final_state, {}, config_.dt_s);
    if (!next.valid)
      return;
    out.final_state = next.state;
    out.poses.push_back(next.state.pose);
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
  out.controls.clear();
  out.active_controls.clear();
  out.branch = branch;
  out.final_state = initial;
  if (!detail::valid_vehicle(initial, config_) || !initial.mode_confirmed || initial.mode_fault ||
      controls.size() != config_.horizon_steps ||
      (branch.switches &&
       (branch.mode == initial.actual_mode || branch.switch_step >= config_.horizon_steps ||
        initial.time_in_mode_s + branch.switch_step * config_.dt_s <
            config_.minimum_mode_dwell_s - 1e-9)))
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
                              controls[branch.switch_step]) < 0.0)
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
    if (continuing_alignment &&
        (step - alignment_begin) * config_.dt_s > config_.confirmation_timeout_s)
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
    out.poses.push_back(next.state.pose);
    ++step;
  }
  out.valid = out.poses.size() == config_.horizon_steps + 1;
}
} // namespace swerve_mppi
