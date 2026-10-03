#pragma once
#include "swerve_mppi/timing.hpp"
namespace swerve_mppi::detail {
struct JointCommandEnvelope {
  std::uint64_t session_id = 0;
  std::uint64_t sequence = 0;
  double issued_at_s = 0;
  JointCommand command;
  // Required metadata: state used to compute, scheduled application, and final
  // admissible start time. Rewrapping an old plan must not refresh source_stamp_s.
  double source_stamp_s = -1;
  double execute_at_s = -1;
  double valid_until_s = -1;
  // Required for guarded execution; absence cannot authorize the queued command.
  std::optional<CommandTask> source_task;
  // Borrowed only for the synchronous body-command update; avoids copying an
  // arbitrary task before workload admission. Owned snapshots remain for tests.
  const CommandTask *source_task_view = nullptr;
  const CommandTask *task() const {
    return source_task_view ? source_task_view : source_task ? &*source_task : nullptr;
  }
};
// The guard reads transport metadata only; joint payloads stay private.
inline CommandEnvelope timing_envelope(const JointCommandEnvelope &command) {
  return {command.session_id,     command.sequence,     command.issued_at_s,   {},
          command.source_stamp_s, command.execute_at_s, command.valid_until_s, std::nullopt};
}
class JointTimedExecutor {
public:
  JointTimedExecutor(const Config &config, std::uint64_t session_id,
                     DriveMode initial_mode = DriveMode::DualAckermann,
                     const TimingLimits &limits = {},
                     std::shared_ptr<const TrajectoryValidator> validator = nullptr);
  TimedExecutionResult update(const std::optional<JointCommandEnvelope> &command,
                              const ControllerInput &latest, double now_s);
  void reset(const VehicleState &recovered, std::uint64_t new_session_id);

private:
  TrajectoryStatus check(const ControllerInput &latest, const ActuationPlan &plan);
  TimedExecutionResult reject_command(const ControllerInput &latest, ExecutionSafetyError reason,
                                      TrajectoryStatus status);
  Config config_;
  std::shared_ptr<const TrajectoryValidator> validator_;
  ActuationModel actuation_;
  RolloutEngine rollout_;
  Trajectory trace_;
  TimingGuard timing_;
  ModeExecutor executor_;
};
} // namespace swerve_mppi::detail
