#include "swerve_mppi/feedback.hpp"

#include "swerve_mppi/model.hpp"
#include "validation.hpp"
#include <cmath>

namespace swerve_mppi {
FeedbackCheck check_feedback(const VehicleState &state, const Config &config) {
  if (!detail::valid_vehicle(state, config))
    return {};
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
} // namespace swerve_mppi
