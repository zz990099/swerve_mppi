#pragma once

#include "swerve_mppi/common/config.hpp"
#include "swerve_mppi/common/types.hpp"

namespace swerve_mppi
{
enum class FeedbackStatus
{
  Valid,
  Invalid,
  Inconsistent
};
struct FeedbackCheck
{
  FeedbackStatus status = FeedbackStatus::Invalid;
  double linear_error_mps = 0;
  double angular_error_radps = 0;
};
// Body-frame twist and linear wheel speeds must represent the same instant.
// Configurable diagnostic tolerance; Valid does not authorize nominal
// modelling.
FeedbackCheck check_feedback(const VehicleState & state, const Config & config);
// Nominal models require agreement to numerical precision (1e-9 in each norm),
// as well as diagnostic admission. Rejects unmodelled residual body motion.
FeedbackCheck check_model_feedback(const VehicleState & state, const Config & config);
}  // namespace swerve_mppi
