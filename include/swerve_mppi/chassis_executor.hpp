#pragma once
#include "swerve_mppi/executor.hpp"
#include <memory>
namespace swerve_mppi {
// Body-command supervisor for offline/reference execution. Integration uses
// TimedExecutor, which additionally validates task, time and the complete stop.
class ChassisExecutor {
public:
  explicit ChassisExecutor(const Config &config, DriveMode initial_mode = DriveMode::DualAckermann);
  ~ChassisExecutor();
  ChassisExecutor(ChassisExecutor &&) noexcept;
  ChassisExecutor &operator=(ChassisExecutor &&) noexcept;
  ExecutionResult update(const Output &command, const VehicleState &measured);
  void reset(const VehicleState &recovered);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace swerve_mppi
