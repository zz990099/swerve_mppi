#pragma once

#include "swerve_mppi/executor.hpp"

namespace swerve_mppi {
struct TimingLimits {
  double max_feedback_age_s = .15;
  double max_command_age_s = .15;
  double period_tolerance_ratio = .25;
};
enum class TimingError {
  None,
  InvalidTime,
  ClockDiscontinuity,
  OffPeriod,
  FeedbackTimeout,
  NonmonotonicFeedback,
  CommandTimeout,
  CommandReplay,
  SessionMismatch,
  MissingCommand
};
struct CommandEnvelope {
  std::uint64_t session_id = 0;
  std::uint64_t sequence = 0;
  double issued_at_s = 0;
  Output command;
};

// All timestamps use one clock. Call check_feedback once per model tick,
// followed by check_command at the same now_s. An error latches until recovery.
class TimingGuard {
public:
  TimingGuard(const Config &config, std::uint64_t session_id, const TimingLimits &limits = {});
  TimingError check_feedback(double measurement_stamp_s, double now_s);
  TimingError check_command(const std::optional<CommandEnvelope> &command, double now_s);
  void reset(std::uint64_t new_session_id);
  std::uint64_t session_id() const { return session_id_; }
  TimingError error() const { return error_; }

private:
  double dt_s_;
  TimingLimits limits_;
  std::uint64_t session_id_;
  std::uint64_t last_sequence_ = 0;
  double last_now_s_ = -1;
  double last_measurement_s_ = -1;
  double last_command_s_ = -1;
  double last_command_tick_s_ = -1;
  TimingError error_ = TimingError::None;
};

struct TimedExecutionResult {
  ExecutionResult execution;
  TimingError timing_error = TimingError::None;
};
// Guarded entry point for queued/transported commands. A missing command is an
// explicit watchdog tick, never implicit permission to reuse the previous Drive.
class TimedExecutor {
public:
  TimedExecutor(const Config &config, std::uint64_t session_id,
                DriveMode initial_mode = DriveMode::DualAckermann, const TimingLimits &limits = {});
  TimedExecutionResult update(const std::optional<CommandEnvelope> &command,
                              const VehicleState &measured, double now_s);
  void reset(const VehicleState &recovered, std::uint64_t new_session_id);

private:
  TimingGuard timing_;
  ModeExecutor executor_;
};
} // namespace swerve_mppi
