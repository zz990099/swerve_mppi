#include "swerve_mppi/feedback/motion_observer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "safety/detail/validation.hpp"
#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi
{
MotionObserver::MotionObserver(const Config & config) : config_(config) { validate(config_); }
MotionAssessment MotionObserver::assess(
  const VehicleState & state, std::int64_t encoder_stamp_ns,
  const std::optional<MotionObservation> & observation) const
{
  if (
    encoder_stamp_ns < 0 || !detail::valid_vehicle(state, config_) ||
    state.stamp_s !=
      std::chrono::duration<double>(std::chrono::nanoseconds(encoder_stamp_ns)).count()) {
    return {};
  }
  if (!observation) {
    MotionAssessment out;
    out.status = MotionStatus::Missing;
    return out;
  }
  const auto & o = *observation;
  if (
    o.stamp_ns < 0 || !std::isfinite(o.velocity.vx) || !std::isfinite(o.velocity.vy) ||
    !std::isfinite(o.velocity.wz) || !std::isfinite(o.linear_error_bound_mps) ||
    o.linear_error_bound_mps < 0 || !std::isfinite(o.angular_error_bound_radps) ||
    o.angular_error_bound_radps < 0 ||
    (o.source != MotionSource::IndependentBody && o.source != MotionSource::EncoderDerived)) {
    return {};
  }
  MotionAssessment out;
  if (o.stamp_ns != encoder_stamp_ns) {
    out.status = MotionStatus::Unsynchronized;
    return out;
  }
  if (o.source != MotionSource::IndependentBody) {
    out.status = MotionStatus::CorrelatedSource;
    return out;
  }
  const auto encoded = Kinematics(config_).forward(state.wheel_speeds, state.steering_angles);
  out.linear_residual_mps = std::hypot(o.velocity.vx - encoded.vx, o.velocity.vy - encoded.vy);
  out.angular_residual_radps = std::abs(o.velocity.wz - encoded.wz);
  out.linear_residual_upper_mps = out.linear_residual_mps + o.linear_error_bound_mps;
  out.angular_residual_upper_radps = out.angular_residual_radps + o.angular_error_bound_radps;
  out.body_speed_upper_mps = std::hypot(o.velocity.vx, o.velocity.vy) + o.linear_error_bound_mps;
  out.body_rate_upper_radps = std::abs(o.velocity.wz) + o.angular_error_bound_radps;
  for (double value :
       {out.linear_residual_upper_mps, out.angular_residual_upper_radps, out.body_speed_upper_mps,
        out.body_rate_upper_radps}) {
    if (!std::isfinite(value)) {
      return {};
    }
  }
  // No tolerance epsilon on declared bounds: numerical agreement is a
  // separate nominal category and never absorbs nonzero uncertainty.
  if (
    out.linear_residual_upper_mps > config_.feedback_linear_tolerance_mps ||
    out.angular_residual_upper_radps > config_.feedback_angular_tolerance_radps) {
    out.status = MotionStatus::EnvelopeExceeded;
  } else if (
    out.linear_residual_mps <= 1e-9 && out.angular_residual_radps <= 1e-9 &&
    o.linear_error_bound_mps == 0 && o.angular_error_bound_radps == 0) {
    out.status = MotionStatus::NominalAgreement;
  } else {
    out.status = MotionStatus::BoundedDisagreement;
  }
  out.body_stationary = out.body_speed_upper_mps <= config_.stopped_linear_mps &&
                        out.body_rate_upper_radps <= config_.stopped_angular_radps;
  out.encoder_stationary =
    std::hypot(encoded.vx, encoded.vy) <= config_.stopped_linear_mps &&
    std::abs(encoded.wz) <= config_.stopped_angular_radps &&
    std::all_of(state.wheel_speeds.begin(), state.wheel_speeds.end(), [&](double v) {
      return std::abs(v) <= config_.stopped_wheel_speed_mps;
    });
  return out;
}
}  // namespace swerve_mppi
