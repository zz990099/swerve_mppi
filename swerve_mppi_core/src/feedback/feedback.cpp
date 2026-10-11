#include "swerve_mppi/feedback/feedback.hpp"

#include <cmath>

#include "safety/detail/validation.hpp"
#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi
{
FeedbackCheck check_feedback(const VehicleState & state, const Config & config)
{
  if (!detail::valid_vehicle(state, config)) {
    return {};
  }
  const auto encoded = Kinematics(config).forward(state.wheel_speeds, state.steering_angles);
  FeedbackCheck out;
  out.linear_error_mps = std::hypot(encoded.vx - state.velocity.vx, encoded.vy - state.velocity.vy);
  out.angular_error_radps = std::abs(encoded.wz - state.velocity.wz);
  out.status = std::isfinite(out.linear_error_mps) && std::isfinite(out.angular_error_radps) &&
                   out.linear_error_mps <= config.feedback_linear_tolerance_mps + 1e-9 &&
                   out.angular_error_radps <= config.feedback_angular_tolerance_radps + 1e-9
                 ? FeedbackStatus::Valid
                 : FeedbackStatus::Inconsistent;
  return out;
}
FeedbackCheck check_model_feedback(const VehicleState & state, const Config & config)
{
  auto out = check_feedback(state, config);
  if (
    out.status == FeedbackStatus::Valid &&
    (out.linear_error_mps > 1e-9 || out.angular_error_radps > 1e-9)) {
    out.status = FeedbackStatus::Inconsistent;
  }
  return out;
}
}  // namespace swerve_mppi
