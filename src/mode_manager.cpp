#include "swerve_mppi/feedback.hpp"
#include "swerve_mppi/mode.hpp"
#include "time_comparison.hpp"
#include "validation.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace swerve_mppi {
ModeManager::ModeManager(const Config &config) : config_(config) { validate(config_); }
void ModeManager::begin(DriveMode target_mode, const Control &intent,
                        const VehicleState &observed) {
  if (active() || check_model_feedback(observed, config_).status != FeedbackStatus::Valid ||
      !observed.mode_confirmed || observed.mode_fault || target_mode == observed.actual_mode ||
      !detail::valid_mode(target_mode) || !std::isfinite(intent.vx) || !std::isfinite(intent.vy) ||
      !std::isfinite(intent.wz))
    throw std::invalid_argument("invalid mode request or active transition");
  const auto previous = std::max(last_request_id_, observed.mode_request_id);
  if (previous == std::numeric_limits<std::uint64_t>::max())
    throw std::overflow_error("mode request IDs exhausted");
  request_ = {previous + 1,
              target_mode,
              DriveModel(config_).steering_for_entry(target_mode, intent, observed.steering_angles),
              {intent.vx, intent.vy, intent.wz}};
  last_request_id_ = request_.id;
  start_s_ = observed.stamp_s;
  last_stamp_s_ = -1.0;
  phase_ = TransitionPhase::Braking;
}

JointCommand ModeManager::update(const VehicleState &observed) {
  JointCommand out;
  out.phase = phase_;
  out.requested_mode = request_.mode;
  out.steering_targets = observed.steering_angles;
  if (check_model_feedback(observed, config_).status != FeedbackStatus::Valid ||
      observed.mode_fault) {
    phase_ = TransitionPhase::Fault;
    out.phase = phase_;
    out.action = Action::SafeStop;
    return out;
  }
  if (phase_ == TransitionPhase::Stable) {
    out.action = Action::Hold;
    return out;
  }
  if (observed.stamp_s < start_s_ || observed.stamp_s <= last_stamp_s_ || observed.mode_fault ||
      phase_ == TransitionPhase::Fault ||
      detail::deadline_exceeded(observed.stamp_s, start_s_, config_.confirmation_timeout_s)) {
    phase_ = TransitionPhase::Fault;
    out.phase = phase_;
    out.action = Action::SafeStop;
    return out;
  }
  last_stamp_s_ = observed.stamp_s;
  if (phase_ == TransitionPhase::Braking) {
    if (is_stopped(observed, config_))
      phase_ = TransitionPhase::AwaitingConfirmation;
    else {
      out.action = Action::Brake;
      return out;
    }
  } else if (observed.actual_mode == request_.mode && observed.mode_confirmed &&
             observed.mode_request_id == request_.id && is_stopped(observed, config_) &&
             detail::steering_aligned(observed.steering_angles, request_.steering_targets,
                                      config_)) {
    phase_ = TransitionPhase::Stable;
    out.phase = phase_;
    out.action = Action::Hold;
    return out;
  }
  out.action = Action::RequestMode;
  out.phase = phase_;
  out.mode_request = request_;
  out.steering_targets = request_.steering_targets;
  return out;
}

void ModeManager::reset() {
  phase_ = TransitionPhase::Stable;
  last_stamp_s_ = -1.0;
  // Never reuse an ID after cancellation/recovery within this controller session.
}
} // namespace swerve_mppi
