#include "swerve_mppi/profile_runner.hpp"

#include "swerve_mppi/feedback.hpp"
#include "time_comparison.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swerve_mppi {
ProfileRunner::ProfileRunner(const Config &config, double watchdog_s)
    : config_(config), watchdog_s_(watchdog_s) {
  validate(config_);
  if (!std::isfinite(watchdog_s) || watchdog_s <= 0)
    throw std::invalid_argument("actuator watchdog must be finite and positive");
}
void ProfileRunner::stop() {
  plan_.reset();
  fault_ = true;
}
bool ProfileRunner::clock_valid(double now_s, double wall_s) {
  if (!std::isfinite(now_s) || now_s < 0 || !std::isfinite(wall_s) || wall_s < 0 ||
      now_s < last_now_s_ || wall_s < last_wall_s_) {
    stop();
    return false;
  }
  last_now_s_ = now_s;
  last_wall_s_ = wall_s;
  return !fault_;
}
bool ProfileRunner::install(const TimedExecutionResult &result, double application_s,
                            double wall_s) {
  if (!clock_valid(application_s, wall_s))
    return false;
  const bool allowed = result.safety_error == ExecutionSafetyError::None ||
                       result.safety_error == ExecutionSafetyError::CommandRejected ||
                       result.safety_error == ExecutionSafetyError::TaskMismatch;
  if (!result.actuation || result.timing_error != TimingError::None || !allowed ||
      (result.safety_error != ExecutionSafetyError::None &&
       result.execution.action != Action::Brake && result.execution.action != Action::Hold) ||
      result.execution.feedback.fault || !result.actuation->compatible_with(config_) ||
      check_feedback(result.actuation->start(), config_).status != FeedbackStatus::Valid ||
      result.actuation->action() != result.execution.action ||
      std::abs(result.actuation->start().stamp_s - application_s) >
          detail::time_tolerance(result.actuation->start().stamp_s, application_s) ||
      (application_s_ >= 0 && application_s <= application_s_)) {
    stop();
    return false;
  }
  // Do not silently recover a missed boundary by installing the next Drive.
  if (plan_ && (detail::deadline_exceeded(application_s, application_s_, plan_->duration_s()) ||
                detail::deadline_exceeded(wall_s, installed_wall_s_, watchdog_s_))) {
    stop();
    return false;
  }
  plan_ = result.actuation;
  application_s_ = application_s;
  installed_wall_s_ = wall_s;
  return true;
}
std::optional<JointTargets> ProfileRunner::sample(double now_s, double wall_s) {
  if (!clock_valid(now_s, wall_s))
    return std::nullopt;
  if (!plan_ || detail::deadline_exceeded(wall_s, installed_wall_s_, watchdog_s_)) {
    stop();
    return std::nullopt;
  }
  if (detail::deadline_exceeded(now_s, application_s_, plan_->duration_s())) {
    stop();
    return std::nullopt;
  }
  const auto targets = plan_->sample(std::clamp(now_s - application_s_, 0.0, plan_->duration_s()));
  if (!targets) {
    stop();
    return std::nullopt;
  }
  JointTargets out;
  out.steering_angles = targets->steering_angles;
  for (std::size_t i = 0; i < 4; ++i)
    out.wheel_angular_speeds[i] = targets->wheel_speeds[i] / config_.wheel_radius_m;
  return out;
}
void ProfileRunner::reset(const VehicleState &recovered) {
  if (check_feedback(recovered, config_).status != FeedbackStatus::Valid || recovered.mode_fault ||
      !recovered.mode_confirmed || !is_stopped(recovered, config_))
    throw std::invalid_argument("profile recovery requires verified stopped/confirmed feedback");
  plan_.reset();
  application_s_ = installed_wall_s_ = last_now_s_ = last_wall_s_ = -1;
  fault_ = false;
}
} // namespace swerve_mppi
