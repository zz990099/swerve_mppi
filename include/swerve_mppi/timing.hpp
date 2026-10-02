#pragma once

#include "swerve_mppi/trajectory_validator.hpp"

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
  MissingCommand,
  InvalidPlanTime,
  SourceTimeout,
  NotYetExecutable,
  ExecutionExpired
};
struct CommandEnvelope {
  std::uint64_t session_id = 0;
  std::uint64_t sequence = 0;
  double issued_at_s = 0;
  Output command;
  // Required metadata: state used to compute, scheduled application, and final
  // admissible start time. Rewrapping an old plan must not refresh source_stamp_s.
  double source_stamp_s = -1;
  double execute_at_s = -1;
  double valid_until_s = -1;
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

enum class ExecutionSafetyError {
  None,
  InvalidContext,
  StateNotCurrent,
  InvalidActuation,
  CommandRejected,
  UnsafeStoppingTrajectory
};
struct TimedExecutionResult {
  ExecutionResult execution;
  TimingError timing_error = TimingError::None;
  ExecutionSafetyError safety_error = ExecutionSafetyError::None;
  TrajectoryStatus rejected_status = TrajectoryStatus::Valid;
  // Present only for a healthy, checked execution. Consume this profile at the
  // actuator rate; endpoint targets alone do not encode the braking/interpolation.
  std::optional<ActuationPlan> actuation;
};
// Guarded entry point for queued/transported commands. A missing command is an
// explicit watchdog tick, never implicit permission to reuse the previous Drive.
// latest.vehicle must describe the execution start (stamp == now_s), with the
// current task/obstacles. Stale raw observations require adapter time alignment.
class TimedExecutor {
public:
  TimedExecutor(const Config &config, std::uint64_t session_id,
                DriveMode initial_mode = DriveMode::DualAckermann, const TimingLimits &limits = {},
                std::shared_ptr<const TrajectoryValidator> validator = nullptr);
  TimedExecutionResult update(const std::optional<CommandEnvelope> &command,
                              const ControllerInput &latest, double now_s);
  void reset(const VehicleState &recovered, std::uint64_t new_session_id);

private:
  TrajectoryStatus check(const ControllerInput &latest, const ActuationPlan &plan);
  Config config_;
  std::shared_ptr<const TrajectoryValidator> validator_;
  ActuationModel actuation_;
  RolloutEngine rollout_;
  Trajectory trace_;
  TimingGuard timing_;
  ModeExecutor executor_;
};
} // namespace swerve_mppi
