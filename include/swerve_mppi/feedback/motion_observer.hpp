#pragma once

#include <limits>

#include "swerve_mppi/feedback/feedback.hpp"

namespace swerve_mppi
{
enum class MotionSource
{
  IndependentBody,
  EncoderDerived
};
// Body-frame velocity at the original source timestamp. Error bounds are
// deterministic norm bounds, not variance, standard deviation or confidence.
struct MotionObservation
{
  Twist2d velocity;
  std::int64_t stamp_ns = -1;
  MotionSource source = MotionSource::EncoderDerived;
  double linear_error_bound_mps = 0;
  double angular_error_bound_radps = 0;
};
enum class MotionStatus
{
  Invalid,
  Missing,
  Unsynchronized,
  CorrelatedSource,
  NominalAgreement,
  BoundedDisagreement,
  EnvelopeExceeded
};
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
// Encoder timestamps must be the original integer stamps, not reconstructed
// from state.stamp_s. An exact common timestamp is required; freshness alone
// does not establish coherence. The observation must be independently sourced.
class MotionObserver
{
public:
  explicit MotionObserver(const Config & config);
  MotionAssessment assess(
    const VehicleState & state, std::int64_t encoder_stamp_ns,
    const std::optional<MotionObservation> & observation) const;

private:
  Config config_;
};
}  // namespace swerve_mppi
