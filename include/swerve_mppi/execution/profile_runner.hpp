#pragma once

#include "swerve_mppi/execution/timing.hpp"

namespace swerve_mppi
{
struct JointTargets
{
  std::array<double, 4> steering_angles{};
  std::array<double, 4> wheel_angular_speeds{};  // rad/s; FL, FR, RL, RR.
};
// Serial high-rate consumer of TimedExecutor results. The actuator callback
// must run independently of planning and use a monotonic wall clock for
// watchdog_s. No target means emergency stop, never permission to retain a
// previous Drive.
class ProfileRunner
{
public:
  explicit ProfileRunner(const Config & config, double watchdog_s = .5);
  bool install(const TimedExecutionResult & result, double application_s, double wall_s);
  std::optional<JointTargets> sample(double now_s, double wall_s);
  bool fault() const { return fault_; }
  // Called only after the owner independently stops/verifies the plant and
  // renews the TimedExecutor session. It cannot reset that protocol itself.
  void reset(const VehicleState & recovered);

private:
  bool clock_valid(double now_s, double wall_s);
  void stop();
  Config config_;
  double watchdog_s_;
  std::optional<ActuationPlan> plan_;
  double application_s_ = -1;
  double installed_wall_s_ = -1;
  double last_now_s_ = -1;
  double last_wall_s_ = -1;
  bool fault_ = false;
};
}  // namespace swerve_mppi
