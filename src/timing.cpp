#include "joint_timing.hpp"
#include "swerve_mppi/feedback.hpp"
#include "time_comparison.hpp"
#include "validation.hpp"
#include <cmath>
#include <stdexcept>

namespace swerve_mppi {
namespace {
bool valid_time(double stamp) { return std::isfinite(stamp) && stamp >= 0; }
} // namespace
CommandTask CommandTask::capture(const ControllerInput &source) {
  return {source.path_id, source.heading_policy, source.reference_path};
}
bool CommandTask::matches(const ControllerInput &latest) const {
  if (path_id != latest.path_id || heading_policy != latest.heading_policy ||
      reference_path.size() != latest.reference_path.size())
    return false;
  for (std::size_t i = 0; i < reference_path.size(); ++i) {
    const auto &a = reference_path[i];
    const auto &b = latest.reference_path[i];
    if (a.x != b.x || a.y != b.y || a.yaw != b.yaw)
      return false;
  }
  return true;
}
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
  if (!valid_time(now) || !valid_time(stamp) || detail::deadline_exceeded(stamp, now, 0))
    return error_ = TimingError::InvalidTime;
  if (last_now_s_ >= 0 && now <= last_now_s_)
    return error_ = TimingError::ClockDiscontinuity;
  if (detail::deadline_exceeded(now, stamp, limits_.max_feedback_age_s))
    return error_ = TimingError::FeedbackTimeout;
  if (last_measurement_s_ >= 0 && stamp <= last_measurement_s_)
    return error_ = TimingError::NonmonotonicFeedback;
  const double tolerance =
      limits_.period_tolerance_ratio * dt_s_ + detail::time_tolerance(now, stamp);
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
  if (!valid_time(now) || last_now_s_ < 0 ||
      std::abs(now - last_now_s_) > detail::time_tolerance(now, last_now_s_))
    return error_ = TimingError::InvalidTime;
  if (!command)
    return error_ = TimingError::MissingCommand;
  const auto &envelope = *command;
  if (envelope.session_id != session_id_)
    return error_ = TimingError::SessionMismatch;
  if (!valid_time(envelope.issued_at_s) || detail::deadline_exceeded(envelope.issued_at_s, now, 0))
    return error_ = TimingError::InvalidTime;
  if (detail::deadline_exceeded(now, envelope.issued_at_s, limits_.max_command_age_s))
    return error_ = TimingError::CommandTimeout;
  if (now <= last_command_tick_s_ || envelope.sequence == 0 ||
      envelope.sequence <= last_sequence_ || envelope.issued_at_s <= last_command_s_)
    return error_ = TimingError::CommandReplay;
  if (!valid_time(envelope.source_stamp_s) || !valid_time(envelope.execute_at_s) ||
      !valid_time(envelope.valid_until_s) ||
      detail::deadline_exceeded(envelope.source_stamp_s, envelope.issued_at_s, 0) ||
      detail::deadline_exceeded(envelope.issued_at_s, envelope.execute_at_s, 0) ||
      detail::deadline_exceeded(envelope.execute_at_s, envelope.valid_until_s, 0))
    return error_ = TimingError::InvalidPlanTime;
  if (detail::deadline_exceeded(now, envelope.source_stamp_s, limits_.max_feedback_age_s))
    return error_ = TimingError::SourceTimeout;
  if (detail::deadline_exceeded(envelope.execute_at_s, now, 0))
    return error_ = TimingError::NotYetExecutable;
  if (detail::deadline_exceeded(now, envelope.valid_until_s, 0))
    return error_ = TimingError::ExecutionExpired;
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
detail::JointTimedExecutor::JointTimedExecutor(const Config &config, std::uint64_t session_id,
                                               DriveMode initial_mode, const TimingLimits &limits,
                                               std::shared_ptr<const TrajectoryValidator> validator)
    : config_(config),
      validator_(validator ? std::move(validator) : std::make_shared<TrajectoryValidator>(config)),
      actuation_(config), rollout_(config), timing_(config, session_id, limits),
      executor_(config, initial_mode) {
  validator_->require_compatible(config_);
}
TrajectoryStatus detail::JointTimedExecutor::check(const ControllerInput &latest,
                                                   const ActuationPlan &plan) {
  rollout_.generate_execution(plan, trace_);
  return validator_->check(latest, trace_);
}
TimedExecutionResult
detail::JointTimedExecutor::update(const std::optional<JointCommandEnvelope> &command,
                                   const ControllerInput &latest, double now_s) {
  const auto &measured = latest.vehicle;
  auto error = timing_.check_feedback(measured.stamp_s, now_s);
  if (error == TimingError::None)
    error = timing_.check_command(
        command ? std::optional<CommandEnvelope>(timing_envelope(*command)) : std::nullopt, now_s);
  JointCommand stop;
  stop.requested_mode = measured.actual_mode;
  if (error != TimingError::None)
    return {executor_.update(stop, measured), error, ExecutionSafetyError::None,
            TrajectoryStatus::Invalid, std::nullopt};
  if (detail::valid_vehicle(measured, config_) &&
      check_model_feedback(measured, config_).status == FeedbackStatus::Inconsistent)
    return {executor_.update(stop, measured), error, ExecutionSafetyError::InconsistentFeedback,
            TrajectoryStatus::Invalid, std::nullopt};
  if (!detail::valid_input(latest, config_))
    return {executor_.update(stop, measured), error, ExecutionSafetyError::InvalidContext,
            TrajectoryStatus::Invalid, std::nullopt};
  // A freshness allowance bounds transport age, not displacement since a sample.
  // This entry point certifies one exact execution-start state. It cannot silently
  // treat a merely recent observation as the current physical pose.
  if (std::abs(measured.stamp_s - now_s) > detail::time_tolerance(measured.stamp_s, now_s))
    return {executor_.update(stop, measured), error, ExecutionSafetyError::StateNotCurrent,
            TrajectoryStatus::Invalid, std::nullopt};

  // Cancellation/fault is independent of task identity. Withheld authorization
  // (or a malformed body packet compiled to SafeStop) must not become a healthy
  // TaskMismatch fallback merely because its originating task was replaced.
  if (command->command.action == Action::SafeStop)
    return {executor_.update(stop, measured), error, ExecutionSafetyError::None,
            TrajectoryStatus::Invalid, std::nullopt};

  // Collision validity alone does not authorize a command for a replaced task.
  // Check the source snapshot before any protocol preview can consume a request.
  const auto *source_task = command->task();
  if (!source_task || !source_task->matches(latest))
    return reject_command(latest, ExecutionSafetyError::TaskMismatch, TrajectoryStatus::Invalid);

  // Preview protocol state transactionally. A rejected request must not consume
  // its ID or leave an unexecuted transition committed in the supervisor.
  auto candidate_executor = executor_;
  auto execution = candidate_executor.update(command->command, measured);
  if (execution.feedback.fault) {
    executor_ = std::move(candidate_executor);
    return {execution, error, ExecutionSafetyError::None, TrajectoryStatus::Invalid, std::nullopt};
  }
  auto plan = actuation_.plan(measured, execution);
  if (!plan)
    return {executor_.update(stop, measured), error, ExecutionSafetyError::InvalidActuation,
            TrajectoryStatus::Invalid, std::nullopt};
  const auto status = check(latest, *plan);
  if (status == TrajectoryStatus::Valid) {
    executor_ = std::move(candidate_executor);
    return {execution, error, ExecutionSafetyError::None, status, std::move(plan)};
  }

  return reject_command(latest, ExecutionSafetyError::CommandRejected, status);
}
TimedExecutionResult detail::JointTimedExecutor::reject_command(const ControllerInput &latest,
                                                                ExecutionSafetyError reason,
                                                                TrajectoryStatus status) {
  const auto &measured = latest.vehicle;
  // The command belongs to an obsolete task or its actual interval/stop is unsafe.
  // Only a freshly checked complete stop authorizes a non-latching fallback.
  JointCommand brake;
  brake.action = Action::Brake;
  brake.requested_mode = measured.actual_mode;
  auto braking_executor = executor_;
  const auto execution = braking_executor.update(brake, measured);
  auto plan = actuation_.plan(measured, execution);
  if (plan && check(latest, *plan) == TrajectoryStatus::Valid) {
    executor_ = std::move(braking_executor);
    return {execution, TimingError::None, reason, status, std::move(plan)};
  }
  JointCommand stop;
  stop.requested_mode = measured.actual_mode;
  return {executor_.update(stop, measured), TimingError::None,
          ExecutionSafetyError::UnsafeStoppingTrajectory, status, std::nullopt};
}
void detail::JointTimedExecutor::reset(const VehicleState &recovered,
                                       std::uint64_t new_session_id) {
  // Check session renewal before mutating execution state. Failed recovery must
  // leave both guards latched; ModeExecutor verifies actual stopped feedback.
  if (new_session_id <= timing_.session_id())
    throw std::invalid_argument("recovery requires a strictly newer execution session");
  executor_.reset(recovered);
  timing_.reset(new_session_id);
}
} // namespace swerve_mppi
