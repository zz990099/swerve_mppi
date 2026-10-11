#pragma once

#include <limits>

#include "swerve_mppi/feedback/feedback.hpp"

namespace swerve_mppi
{
struct MotionAssessment
{
  MotionStatus status = MotionStatus::Invalid;
  double linear_residual_mps = std::numeric_limits<double>::infinity();
  double angular_residual_radps = std::numeric_limits<double>::infinity();
  double linear_residual_upper_mps = std::numeric_limits<double>::infinity();
  double angular_residual_upper_radps = std::numeric_limits<double>::infinity();
  double body_speed_upper_mps = std::numeric_limits<double>::infinity();
  double body_rate_upper_radps = std::numeric_limits<double>::infinity();
  bool body_stationary = false;
  bool encoder_stationary = false;
};
// Stateless diagnostic sidecar: never changes VehicleState, authorizes Drive,
// certifies a stopping trajectory, latches a fault or resets an executor.
// Source timestamps stay integer nanoseconds. Configured pairing and age bounds
// establish coherence; observations are never relabelled to planning time.
class MotionObserver
{
public:
  explicit MotionObserver(const Config & config);
  MotionAssessment assess(
    const VehicleState & state, TimestampNs planning_stamp_ns,
    const std::optional<MotionObservation> & observation) const;

private:
  Config config_;
};
}  // namespace swerve_mppi
