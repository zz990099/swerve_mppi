#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "common/detail/time_comparison.hpp"
#include "safety/detail/validation.hpp"
#include "swerve_mppi/feedback/feedback.hpp"
#include "swerve_mppi/planning/mode.hpp"

namespace swerve_mppi
{
namespace
{
bool same_entry_geometry(
  const std::array<double, 4> & accepted, const std::array<double, 4> & planned)
{
  // The chassis executor chooses and freezes mechanical positions at first
  // accepted application. A queued body request may select a different signed
  // wheel representation than prediction. Acknowledgement checks the same
  // rolling lines; it never commands a wrapped shortcut across a hard stop.
  constexpr double pi = 3.14159265358979323846;
  for (std::size_t i = 0; i < accepted.size(); ++i) {
    if (!(std::abs(std::remainder(accepted[i] - planned[i], pi)) <= 1e-7)) {
      return false;
    }
  }
  return true;
}
}  // namespace
ModeManager::ModeManager(const Config & config) : config_(config) { validate(config_); }
void ModeManager::begin(
  DriveMode target_mode, const Control & intent, const VehicleState & observed)
{
  if (
    active() || check_model_feedback(observed, config_).status != FeedbackStatus::Valid ||
    !observed.mode_confirmed || observed.mode_fault || target_mode == observed.actual_mode ||
    !DriveModel(config_).feasible(intent, target_mode)) {
    throw std::invalid_argument("invalid mode request or active transition");
  }
  const auto previous = std::max(last_request_id_, observed.mode_request_id);
  if (previous == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("mode request IDs exhausted");
  }
  request_ = {
    previous + 1,
    target_mode,
    DriveModel(config_).steering_for_entry(target_mode, intent, observed.steering_angles),
    {intent.vx, intent.vy, intent.wz}};
  last_request_id_ = request_.id;
  accepted_request_.reset();
  start_s_ = observed.stamp_s;
  last_stamp_s_ = -1.0;
  phase_ = TransitionPhase::Braking;
}

JointCommand ModeManager::update(const VehicleState & observed)
{
  JointCommand out;
  out.phase = phase_;
  out.requested_mode = request_.mode;
  out.steering_targets = observed.steering_angles;
  if (
    check_model_feedback(observed, config_).status != FeedbackStatus::Valid ||
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
  if (
    observed.stamp_s < start_s_ || observed.stamp_s <= last_stamp_s_ || observed.mode_fault ||
    phase_ == TransitionPhase::Fault ||
    detail::deadline_exceeded(observed.stamp_s, start_s_, config_.confirmation_timeout_s)) {
    phase_ = TransitionPhase::Fault;
    out.phase = phase_;
    out.action = Action::SafeStop;
    return out;
  }
  last_stamp_s_ = observed.stamp_s;
  const auto & accepted = observed.accepted_mode_request;
  const auto invalid_acceptance = [&]() {
    phase_ = TransitionPhase::Fault;
    out.phase = phase_;
    out.action = Action::SafeStop;
    return out;
  };
  if (accepted_request_ && (!accepted || !detail::same_request(*accepted, *accepted_request_))) {
    return invalid_acceptance();
  }
  if (observed.mode_request_id == request_.id) {
    if (
      !accepted || accepted->mode != request_.mode ||
      accepted->entry_velocity.vx != request_.entry_velocity.vx ||
      accepted->entry_velocity.vy != request_.entry_velocity.vy ||
      accepted->entry_velocity.wz != request_.entry_velocity.wz ||
      !same_entry_geometry(accepted->steering_targets, request_.steering_targets)) {
      return invalid_acceptance();
    }
    accepted_request_ = accepted;
  }
  if (phase_ == TransitionPhase::Braking) {
    if (is_stopped(observed, config_)) {
      phase_ = TransitionPhase::AwaitingConfirmation;
    } else {
      out.action = Action::Brake;
      return out;
    }
  } else if (
    observed.actual_mode == request_.mode && observed.mode_confirmed && accepted_request_ &&
    observed.mode_request_id == request_.id && is_stopped(observed, config_) &&
    detail::steering_aligned(
      observed.steering_angles, accepted_request_->steering_targets, config_)) {
    phase_ = TransitionPhase::Stable;
    out.phase = phase_;
    out.action = Action::Hold;
    return out;
  }
  out.action = Action::RequestMode;
  out.phase = phase_;
  out.mode_request = request_;
  // Before acceptance predict from the current snapshot, just like compilation.
  // Afterwards check the exact committed interval, never a new nearest angle.
  out.steering_targets =
    accepted_request_
      ? accepted_request_->steering_targets
      : DriveModel(config_).steering_for_entry(
          request_.mode,
          {request_.entry_velocity.vx, request_.entry_velocity.vy, request_.entry_velocity.wz},
          observed.steering_angles);
  out.mode_request->steering_targets = out.steering_targets;
  return out;
}

void ModeManager::reset()
{
  phase_ = TransitionPhase::Stable;
  last_stamp_s_ = -1.0;
  accepted_request_.reset();
  // Never reuse an ID after cancellation/recovery within this controller
  // session.
}
}  // namespace swerve_mppi
