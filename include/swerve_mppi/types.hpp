#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace swerve_mppi {

enum class DriveMode { DualAckermann, Spin, Crab };
enum class TransitionPhase { Stable, Braking, Aligning, AwaitingConfirmation, Fault };
enum class Action { Drive, Brake, RequestMode, Hold, SafeStop };

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Twist2d {
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
};

struct VehicleState {
  Pose2d pose;
  Twist2d velocity;
  std::array<double, 4> steering_angles{};
  // Linear rolling m/s in FL, FR, RL, RR order; never raw encoder rad/s.
  std::array<double, 4> wheel_speeds{};
  DriveMode actual_mode = DriveMode::DualAckermann;
  bool mode_confirmed = true;
  bool mode_fault = false;
  // Echo of the executor's active/last completed request; zero means startup.
  std::uint64_t mode_request_id = 0;
  double time_in_mode_s = 0.0;
  double stamp_s = 0.0;
};

struct CircleObstacle {
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
};

struct ControllerInput {
  VehicleState vehicle;
  std::vector<Pose2d> reference_path;
  std::vector<CircleObstacle> obstacles;
};

struct Control {
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
};

struct ModeRequest {
  std::uint64_t id = 0;
  DriveMode mode = DriveMode::DualAckermann;
  // Frozen mechanical positions, including the intended Crab entry direction.
  std::array<double, 4> steering_targets{};
};

struct ModeFeedback {
  DriveMode actual_mode = DriveMode::DualAckermann;
  bool confirmed = true;
  bool fault = false;
  std::uint64_t request_id = 0;
  double time_in_mode_s = 0.0;
};

struct Output {
  Action action = Action::SafeStop;
  DriveMode requested_mode = DriveMode::DualAckermann;
  TransitionPhase phase = TransitionPhase::Stable;
  Twist2d body_command;
  std::array<double, 4> steering_targets{};
  std::array<double, 4> wheel_speed_targets{};
  // Present on RequestMode only; retries preserve every field.
  std::optional<ModeRequest> mode_request;
  double selected_cost = 0.0;
  double keep_cost = 0.0;
  std::size_t feasible_rollouts = 0;
};

} // namespace swerve_mppi
