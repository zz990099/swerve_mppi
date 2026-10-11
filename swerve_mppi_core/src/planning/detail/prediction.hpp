#pragma once
#include "swerve_mppi/common/types.hpp"

namespace swerve_mppi::detail
{
enum class Action
{
  Drive,
  Brake,
  RequestMode,
  Hold,
  SafeStop
};
// Private prediction result; never sent to a chassis.
struct Prediction : PlanningDiagnostics
{
  Action action = Action::SafeStop;
  DriveMode requested_mode = DriveMode::DualAckermann;
  // Drive: endpoint forward kinematics; joints interpolate over the full tick.
  Twist2d body_command;
  Twist2d velocity_intent;  // Nominal target, not the FK endpoint after rate
                            // limiting.
  std::array<double, 4> steering_targets{};
  std::array<double, 4> wheel_speed_targets{};
  // Present on RequestMode only; retries preserve every field.
  std::optional<AcceptedModeRequest> mode_request;
};

}  // namespace swerve_mppi::detail
