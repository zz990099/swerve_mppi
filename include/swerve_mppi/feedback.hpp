#pragma once

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/types.hpp"

namespace swerve_mppi {
enum class FeedbackStatus { Valid, Invalid, Inconsistent };
struct FeedbackCheck {
  FeedbackStatus status = FeedbackStatus::Invalid;
  double linear_error_mps = 0;
  double angular_error_radps = 0;
};
// Body-frame twist and linear wheel speeds must represent the same instant.
// This admission check detects disagreement; it is not a slip uncertainty model.
FeedbackCheck check_feedback(const VehicleState &state, const Config &config);
} // namespace swerve_mppi
