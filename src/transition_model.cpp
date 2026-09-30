#include "swerve_mppi/model.hpp"

#include "validation.hpp"
#include <algorithm>
#include <cmath>

namespace swerve_mppi {
TransitionModel::TransitionModel(const Config &config) : config_(config), model_(config) {}
double TransitionModel::rollout(VehicleState &state, DriveMode target_mode, std::size_t &steps,
                                std::size_t maximum, std::vector<Pose2d> *trace) const {
  if (!detail::valid_vehicle(state, config_) || steps >= maximum ||
      (target_mode != DriveMode::DualAckermann && target_mode != DriveMode::Spin &&
       target_mode != DriveMode::Crab))
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
    if (trace)
      trace->push_back(state.pose);
  }
  state.velocity = {};
  state.wheel_speeds.fill(0.0);
  const auto angles = model_.steering_for_mode(target_mode, state.steering_angles);
  if (config_.alignment_min_s > (maximum - steps) * config_.dt_s)
    return -1.0;
  const std::size_t minimum =
      static_cast<std::size_t>(std::ceil(config_.alignment_min_s / config_.dt_s));
  std::size_t aligned_steps = 0;
  while (true) {
    bool aligned = true;
    for (std::size_t i = 0; i < 4; ++i)
      if (std::abs(angles[i] - state.steering_angles[i]) > config_.steering_tolerance_rad)
        aligned = false;
    if (aligned && aligned_steps >= minimum)
      break;
    if (steps >= maximum)
      return -1.0;
    for (std::size_t i = 0; i < 4; ++i) {
      const double delta = angles[i] - state.steering_angles[i];
      const double limit = config_.max_steer_rate_radps * config_.dt_s;
      state.steering_angles[i] += std::clamp(delta, -limit, limit);
    }
    state.stamp_s += config_.dt_s;
    ++steps;
    ++aligned_steps;
    if (trace)
      trace->push_back(state.pose);
  }
  if (config_.confirmation_prediction_s > (maximum - steps) * config_.dt_s)
    return -1.0;
  const std::size_t confirmation =
      static_cast<std::size_t>(std::ceil(config_.confirmation_prediction_s / config_.dt_s));
  // A zero-delay transition still consumes a tick, guaranteeing rollout progress.
  const std::size_t wait = std::max(confirmation, steps == begin ? std::size_t{1} : std::size_t{0});
  if (wait > maximum - steps)
    return -1.0;
  state.stamp_s += wait * config_.dt_s;
  steps += wait;
  if (trace)
    for (std::size_t i = 0; i < wait; ++i)
      trace->push_back(state.pose);
  state.actual_mode = target_mode;
  state.mode_confirmed = true;
  state.time_in_mode_s = 0.0;
  return (steps - begin) * config_.dt_s;
}
} // namespace swerve_mppi
