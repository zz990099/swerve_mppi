#include "swerve_mppi/model.hpp"

#include "motion_profile.hpp"
#include "stopping_motion.hpp"
#include "time_comparison.hpp"
#include "validation.hpp"
#include <algorithm>
#include <cmath>

namespace swerve_mppi {
TransitionModel::TransitionModel(const Config &config) : config_(config), model_(config) {}
double TransitionModel::rollout(VehicleState &state, DriveMode target_mode, std::size_t &steps,
                                std::size_t maximum, std::vector<Pose2d> *trace,
                                const Control &entry_intent, std::vector<double> *sweep_margins,
                                double *position_error_m) const {
  double local_error = 0;
  double &error = position_error_m ? *position_error_m : local_error;
  if (!detail::valid_vehicle(state, config_) || steps >= maximum ||
      (target_mode != DriveMode::DualAckermann && target_mode != DriveMode::Spin &&
       target_mode != DriveMode::Crab) ||
      !std::isfinite(entry_intent.vx) || !std::isfinite(entry_intent.vy) ||
      !std::isfinite(entry_intent.wz))
    return -1.0;
  const std::size_t begin = steps;
  while (!is_stopped(state, config_)) {
    if (steps >= maximum)
      return -1.0;
    auto next = model_.step(state, {}, config_.dt_s);
    if (!next.valid)
      return -1.0;
    state = next.state;
    ++steps;
    detail::append_motion(next, trace, sweep_margins, error);
  }
  const auto angles = model_.steering_for_entry(target_mode, entry_intent, state.steering_angles);
  const auto minimum =
      detail::duration_ticks(config_.alignment_min_s, config_.dt_s, maximum - steps);
  if (!minimum)
    return -1.0;
  std::size_t aligned_steps = 0;
  while (true) {
    bool aligned = true;
    for (std::size_t i = 0; i < 4; ++i)
      if (std::abs(angles[i] - state.steering_angles[i]) > config_.steering_tolerance_rad)
        aligned = false;
    if (aligned && aligned_steps >= *minimum)
      break;
    if (steps >= maximum)
      return -1.0;
    const auto next = detail::stopping_step(state, angles, config_, config_.dt_s);
    state = next.state;
    ++steps;
    ++aligned_steps;
    detail::append_motion(next, trace, sweep_margins, error);
  }
  const auto confirmation =
      detail::duration_ticks(config_.confirmation_prediction_s, config_.dt_s, maximum - steps);
  if (!confirmation)
    return -1.0;
  // Even immediate transport needs an executor confirmation Hold, followed by
  // the manager's measured-feedback handover Hold. The configured allowance
  // covers these cycles and may reserve additional feedback/transport latency.
  const std::size_t wait = std::max(*confirmation, std::size_t{2});
  if (wait > maximum - steps)
    return -1.0;
  // The deadline applies at receipt of confirmation, before the final handover
  // cycle ends. First Drive may occur one tick after that receipt deadline.
  if (detail::duration_exceeded((steps + wait - 1 - begin) * config_.dt_s,
                                config_.confirmation_timeout_s))
    return -1.0;
  steps += wait;
  for (std::size_t i = 0; i < wait; ++i) {
    const auto next = detail::stopping_step(state, angles, config_, config_.dt_s);
    state = next.state;
    detail::append_motion(next, trace, sweep_margins, error);
  }
  state.actual_mode = target_mode;
  state.mode_confirmed = true;
  state.time_in_mode_s = 0.0;
  return (steps - begin) * config_.dt_s;
}
} // namespace swerve_mppi
