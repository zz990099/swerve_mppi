#include <algorithm>
#include <cmath>

#include "common/detail/time_comparison.hpp"
#include "model/detail/nominal_motion.hpp"
#include "safety/detail/validation.hpp"
#include "swerve_mppi/feedback/feedback.hpp"
#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi
{
TransitionModel::TransitionModel(const Config & config) : config_(config), model_(config) {}
double TransitionModel::rollout(
  VehicleState & state, DriveMode target_mode, std::size_t & steps, std::size_t maximum,
  std::vector<Pose2d> * trace, const Control & entry_intent, std::vector<double> * sweep_margins,
  double * position_error_m) const
{
  double local_error = 0;
  double & error = position_error_m ? *position_error_m : local_error;
  if (
    check_model_feedback(state, config_).status != FeedbackStatus::Valid || steps >= maximum ||
    (target_mode != DriveMode::DualAckermann && target_mode != DriveMode::Spin &&
     target_mode != DriveMode::Crab) ||
    !std::isfinite(entry_intent.vx) || !std::isfinite(entry_intent.vy) ||
    !std::isfinite(entry_intent.wz) ||
    (target_mode == DriveMode::DualAckermann &&
     (entry_intent.vy != 0 || (entry_intent.vx == 0 && entry_intent.wz != 0))) ||
    (target_mode == DriveMode::Spin && (entry_intent.vx != 0 || entry_intent.vy != 0)) ||
    (target_mode == DriveMode::Crab && entry_intent.wz != 0)) {
    return -1.0;
  }
  const std::size_t begin = steps;
  const auto angles = model_.steering_for_entry(target_mode, entry_intent, state.steering_angles);
  auto memory = model_.alignment_seed(state, angles);
  do {
    if (steps >= maximum) return -1.0;
    const auto next = model_.step(state, {}, config_.model_period_s, memory);
    if (!next.valid) return -1.0;
    state = next.state;
    memory = next.prediction;
    ++steps;
    detail::append_motion(next, trace, sweep_margins, error);
  } while (memory.phase != TransitionPhase::Stable);
  const auto confirmation = detail::duration_ticks(
    config_.confirmation_prediction_s, config_.model_period_s, maximum - steps);
  if (!confirmation) {
    return -1.0;
  }
  // Reserve observation/manager handover time after nominal mechanical completion.
  // This prediction never confirms a real request.
  const std::size_t wait = std::max(*confirmation, std::size_t{2});
  if (wait > maximum - steps) {
    return -1.0;
  }
  // The deadline applies at receipt of confirmation, before the final handover
  // cycle ends. First Drive may occur one tick after that receipt deadline.
  if (detail::duration_exceeded(
        (steps + wait - 1 - begin) * config_.model_period_s, config_.confirmation_timeout_s)) {
    return -1.0;
  }
  steps += wait;
  for (std::size_t i = 0; i < wait; ++i) {
    const auto next = model_.step(state, {}, config_.model_period_s, memory);
    if (!next.valid) return -1.0;
    memory = next.prediction;
    state = next.state;
    detail::append_motion(next, trace, sweep_margins, error);
  }
  state.actual_mode = target_mode;
  // A hypothetical transition cannot echo an actual chassis acceptance.
  state.accepted_mode_request.reset();
  state.mode_confirmed = true;
  state.time_in_mode_s = 0.0;
  return (steps - begin) * config_.model_period_s;
}
}  // namespace swerve_mppi
