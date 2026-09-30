#include "swerve_mppi/rollout.hpp"
#include "validation.hpp"
#include <cmath>

namespace swerve_mppi {
RolloutEngine::RolloutEngine(const Config &config)
    : config_(config), model_(config), transition_(config) {}
Trajectory RolloutEngine::generate(const VehicleState &initial, const Branch &branch,
                                   const std::vector<Control> &controls) const {
  Trajectory out;
  out.branch = branch;
  out.final_state = initial;
  if (!detail::valid_vehicle(initial, config_) || !initial.mode_confirmed || initial.mode_fault ||
      controls.size() != config_.horizon_steps ||
      (branch.switches &&
       (branch.mode == initial.actual_mode || branch.switch_step >= config_.horizon_steps ||
        initial.time_in_mode_s + branch.switch_step * config_.dt_s <
            config_.minimum_mode_dwell_s - 1e-9)))
    return out;
  out.poses.reserve(config_.horizon_steps + 1);
  out.poses.push_back(initial.pose);
  out.controls.resize(controls.size());
  out.active_controls.assign(controls.size(), false);
  std::size_t step = 0;
  bool switched = false;
  while (step < config_.horizon_steps) {
    if (branch.switches && !switched && step == branch.switch_step) {
      std::vector<Pose2d> trace;
      if (transition_.rollout(out.final_state, branch.mode, step, config_.horizon_steps, &trace) <
          0.0)
        return out;
      out.poses.insert(out.poses.end(), trace.begin(), trace.end());
      switched = true;
      continue;
    }
    const DriveMode mode = switched ? branch.mode : initial.actual_mode;
    if (!std::isfinite(controls[step].vx) || !std::isfinite(controls[step].vy) ||
        !std::isfinite(controls[step].wz))
      return out;
    const Control control = model_.project(controls[step], mode);
    const auto next = model_.step(out.final_state, control, config_.dt_s);
    if (!next.valid)
      return out;
    out.final_state = next.state;
    out.controls[step] = control;
    out.active_controls[step] = true;
    out.poses.push_back(next.state.pose);
    ++step;
  }
  out.valid = out.poses.size() == config_.horizon_steps + 1;
  return out;
}
} // namespace swerve_mppi
