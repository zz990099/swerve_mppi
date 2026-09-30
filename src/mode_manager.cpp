#include "swerve_mppi/mode.hpp"
#include "validation.hpp"
#include <stdexcept>
namespace swerve_mppi {
ModeManager::ModeManager(const Config &config) : config_(config) { validate(config_); }
void ModeManager::begin(DriveMode target_mode, double now_s) {
  if (!std::isfinite(now_s) || now_s < 0.0 ||
      (target_mode != DriveMode::DualAckermann && target_mode != DriveMode::Spin &&
       target_mode != DriveMode::Crab))
    throw std::invalid_argument("invalid mode request");
  target_mode_ = target_mode;
  start_s_ = now_s;
  phase_ = TransitionPhase::Braking;
}

Output ModeManager::update(const VehicleState &observed) {
  Output out;
  out.phase = phase_;
  out.requested_mode = target_mode_;
  if (!detail::valid_vehicle(observed, config_) || observed.stamp_s < start_s_ ||
      observed.mode_fault || phase_ == TransitionPhase::Fault ||
      observed.stamp_s - start_s_ > config_.confirmation_timeout_s) {
    phase_ = TransitionPhase::Fault;
    out.phase = phase_;
    out.action = Action::SafeStop;
    return out;
  }
  if (phase_ == TransitionPhase::Braking) {
    if (is_stopped(observed, config_)) {
      phase_ = TransitionPhase::AwaitingConfirmation;
      out.action = Action::RequestMode;
    } else {
      out.action = Action::Brake;
    }
  } else if (phase_ == TransitionPhase::AwaitingConfirmation) {
    if (observed.actual_mode == target_mode_ && observed.mode_confirmed &&
        is_stopped(observed, config_)) {
      phase_ = TransitionPhase::Stable;
      out.action = Action::Hold;
    } else {
      out.action = Action::RequestMode;
    }
  } else {
    out.action = Action::Hold;
  }
  out.phase = phase_;
  return out;
}

void ModeManager::reset() { phase_ = TransitionPhase::Stable; }

} // namespace swerve_mppi
