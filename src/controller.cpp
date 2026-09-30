#include "swerve_mppi/controller.hpp"

#include "validation.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_mppi {
Controller::Controller(const Config &config)
    : config_(config), model_(config), optimizer_(config), scheduler_(config),
      mode_manager_(config) {}

Output Controller::compute(const ControllerInput &input) {
  Output stop;
  stop.requested_mode = input.vehicle.actual_mode;
  stop.phase = mode_manager_.phase();
  if (!detail::valid_input(input, config_) ||
      (last_stamp_s_ >= 0.0 && input.vehicle.stamp_s <= last_stamp_s_))
    return stop;
  stop.steering_targets = input.vehicle.steering_angles;
  last_stamp_s_ = input.vehicle.stamp_s;
  if (mode_manager_.active())
    return mode_manager_.update(input.vehicle);
  if (!input.vehicle.mode_confirmed || input.vehicle.mode_fault)
    return stop;
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
    if (input.vehicle.mode_request_id == std::numeric_limits<std::uint64_t>::max())
      return stop;
    try {
      mode_manager_.begin(best.branch.mode, best.controls.front(), input.vehicle);
    } catch (const std::overflow_error &) {
      return stop;
    }
    optimizer_.reset();
    auto out = mode_manager_.update(input.vehicle);
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
    out.action = is_stopped(input.vehicle, config_) ? Action::Hold : Action::Brake;
  } else {
    out.action = Action::Drive;
    out.body_command = preview.state.velocity;
    out.wheel_speed_targets = preview.wheel_speed_targets;
    optimizer_.accept(best, input.vehicle.actual_mode);
  }
  return out;
}
void Controller::reset() {
  optimizer_.reset();
  mode_manager_.reset();
  last_stamp_s_ = -1.0;
}
} // namespace swerve_mppi
