#pragma once

#include "swerve_mppi/executor.hpp"
#include <optional>

namespace swerve_mppi {
struct ActuatorTargets {
  std::array<double, 4> steering_angles{};
  // Linear rolling m/s, FL, FR, RL, RR. Convert to joint rad/s in the adapter.
  std::array<double, 4> wheel_speeds{};
};

// A checked single-tick reference, not a prediction of an uncalibrated servo.
// Drive is affine over the full tick. Stopping scales all wheels proportionally
// to zero before any Hold/RequestMode steering alignment begins.
class ActuationPlan {
public:
  const VehicleState &start() const { return start_; }
  const StepResult &endpoint() const { return endpoint_; }
  Action action() const { return action_; }
  double duration_s() const { return duration_s_; }
  double braking_duration_s() const { return braking_duration_s_; }
  bool compatible_with(const Config &config) const;
  // No extrapolation: elapsed_s must be finite and inside [0, duration_s()].
  std::optional<ActuatorTargets> sample(double elapsed_s) const;

private:
  friend class ActuationModel;
  ActuationPlan() = default;
  VehicleState start_;
  StepResult endpoint_;
  Action action_ = Action::SafeStop;
  double duration_s_ = 0;
  double braking_duration_s_ = 0;
  double steering_rate_radps_ = 0;
  bool may_align_ = false;
  std::array<double, 22> model_parameters_{};
};

class ActuationModel {
public:
  explicit ActuationModel(const Config &config);
  // SafeStop disables drive; its physical stopping curve is unspecified and
  // deliberately has no certified plan. Use an independent actuator watchdog.
  std::optional<ActuationPlan> plan(const VehicleState &measured,
                                  const ExecutionResult &execution) const;
  // Nominal non-driving interval for planning-side safety checks. Preserves
  // measured protocol feedback; it neither confirms a mode nor authorizes an
  // executor request. Validate the returned interval/stop against constraints.
  std::optional<ActuationPlan> plan_stopping(
      const VehicleState &measured, Action action,
      const std::array<double, 4> &steering_targets) const;

private:
  Config config_;
  Kinematics kinematics_;
};
} // namespace swerve_mppi
