#pragma once

#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi
{

// Transport-independent reference supervisor, not an actuator/dynamics
// simulator. The caller supplies fresh measured body/joint state. Use
// TimedExecutor for integration-time trajectory/timing/task checks and sample
// its ActuationPlan; raw endpoint targets do not encode the full actuator
// reference. Zero drive never erases a mode.
class ModeExecutor
{
public:
  explicit ModeExecutor(const Config & config, DriveMode initial_mode = DriveMode::DualAckermann);
  ExecutionResult update(const JointCommand & command, const VehicleState & measured);
  // Deliberate recovery: requires independently verified stopped, confirmed
  // state. Preserves the request high-water mark to reject delayed pre-reset
  // requests.
  void reset(const VehicleState & recovered);

private:
  Config config_;
  DriveMode actual_mode_;
  TransitionPhase phase_ = TransitionPhase::Stable;
  std::optional<JointModeRequest> request_;
  std::uint64_t last_request_id_ = 0;
  double last_stamp_s_ = -1.0;
  double start_s_ = 0.0;
  double alignment_start_s_ = -1.0;
  double confirmed_s_ = -1.0;
  bool initialized_ = false;
  std::array<double, 4> last_steering_{};
};

}  // namespace swerve_mppi
