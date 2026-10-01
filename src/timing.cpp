#include "swerve_mppi/timing.hpp"
#include <cmath>
#include <stdexcept>

namespace swerve_mppi {
namespace {
bool valid_time(double stamp) { return std::isfinite(stamp) && stamp >= 0; }
constexpr double kTolerance = 1e-9;
} // namespace
TimingGuard::TimingGuard(const Config &config, std::uint64_t session_id, const TimingLimits &limits)
    : dt_s_(config.dt_s), limits_(limits), session_id_(session_id) {
  validate(config);
  if (session_id == 0 || !std::isfinite(limits.max_feedback_age_s) ||
      limits.max_feedback_age_s <= 0 || !std::isfinite(limits.max_command_age_s) ||
      limits.max_command_age_s <= 0 || !std::isfinite(limits.period_tolerance_ratio) ||
      limits.period_tolerance_ratio < 0 || limits.period_tolerance_ratio >= 1)
    throw std::invalid_argument("invalid timing limits or zero execution session");
}
TimingError TimingGuard::check_feedback(double stamp, double now) {
  if (error_ != TimingError::None)
    return error_;
  if (!valid_time(now) || !valid_time(stamp) || stamp > now + kTolerance)
    return error_ = TimingError::InvalidTime;
  if (last_now_s_ >= 0 && now <= last_now_s_)
    return error_ = TimingError::ClockDiscontinuity;
  if (now - stamp > limits_.max_feedback_age_s + kTolerance)
    return error_ = TimingError::FeedbackTimeout;
  if (last_measurement_s_ >= 0 && stamp <= last_measurement_s_)
    return error_ = TimingError::NonmonotonicFeedback;
  const double tolerance = limits_.period_tolerance_ratio * dt_s_ + kTolerance;
  if (last_now_s_ >= 0 && (std::abs(now - last_now_s_ - dt_s_) > tolerance ||
                           std::abs(stamp - last_measurement_s_ - dt_s_) > tolerance))
    return error_ = TimingError::OffPeriod;
  last_now_s_ = now;
  last_measurement_s_ = stamp;
  return TimingError::None;
}
TimingError TimingGuard::check_command(const std::optional<CommandEnvelope> &command, double now) {
  if (error_ != TimingError::None)
    return error_;
  if (!valid_time(now) || last_now_s_ < 0 || std::abs(now - last_now_s_) > kTolerance)
    return error_ = TimingError::InvalidTime;
  if (!command)
    return error_ = TimingError::MissingCommand;
  const auto &envelope = *command;
  if (envelope.session_id != session_id_)
    return error_ = TimingError::SessionMismatch;
  if (!valid_time(envelope.issued_at_s) || envelope.issued_at_s > now + kTolerance)
    return error_ = TimingError::InvalidTime;
  if (now - envelope.issued_at_s > limits_.max_command_age_s + kTolerance)
    return error_ = TimingError::CommandTimeout;
  if (now <= last_command_tick_s_ || envelope.sequence == 0 ||
      envelope.sequence <= last_sequence_ || envelope.issued_at_s <= last_command_s_)
    return error_ = TimingError::CommandReplay;
  last_sequence_ = envelope.sequence;
  last_command_s_ = envelope.issued_at_s;
  last_command_tick_s_ = now;
  return TimingError::None;
}
void TimingGuard::reset(std::uint64_t new_session_id) {
  if (new_session_id <= session_id_)
    throw std::invalid_argument("recovery requires a strictly newer execution session");
  session_id_ = new_session_id;
  last_sequence_ = 0;
  last_now_s_ = last_measurement_s_ = last_command_s_ = -1;
  last_command_tick_s_ = -1;
  error_ = TimingError::None;
}
TimedExecutor::TimedExecutor(const Config &config, std::uint64_t session_id, DriveMode initial_mode,
                             const TimingLimits &limits)
    : timing_(config, session_id, limits), executor_(config, initial_mode) {}
TimedExecutionResult TimedExecutor::update(const std::optional<CommandEnvelope> &command,
                                           const VehicleState &measured, double now_s) {
  auto error = timing_.check_feedback(measured.stamp_s, now_s);
  if (error == TimingError::None)
    error = timing_.check_command(command, now_s);
  Output stop;
  stop.requested_mode = measured.actual_mode;
  const auto execution =
      executor_.update(error == TimingError::None ? command->command : stop, measured);
  return {execution, error};
}
void TimedExecutor::reset(const VehicleState &recovered, std::uint64_t new_session_id) {
  // Check session renewal before mutating execution state. Failed recovery must
  // leave both guards latched; ModeExecutor verifies actual stopped feedback.
  if (new_session_id <= timing_.session_id())
    throw std::invalid_argument("recovery requires a strictly newer execution session");
  executor_.reset(recovered);
  timing_.reset(new_session_id);
}
} // namespace swerve_mppi
