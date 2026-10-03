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
enum class NavigationStatus {
  Tracking,
  ApproachingGoal,
  AligningGoal,
  Settling,
  Complete,
  Fault,
  Waiting
};
enum class PathHeadingPolicy { FollowPath, GoalOnly };

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

struct TrackingContext {
  Pose2d goal;
  double remaining_length_m = 0.0;
  double speed_limit_mps = 0.0;
  bool terminal = false;
  PathHeadingPolicy heading_policy = PathHeadingPolicy::FollowPath;
};

struct ControllerInput {
  VehicleState vehicle;
  std::vector<Pose2d> reference_path;
  std::vector<CircleObstacle> obstacles;
  // Change this ID to restart an identical path as a new task. Geometry changes
  // are also detected automatically. Keep the full path stable between replans.
  std::uint64_t path_id = 0;
  PathHeadingPolicy heading_policy = PathHeadingPolicy::FollowPath;
  // Populated by Controller for Optimizer/Critic calls; Controller overwrites
  // caller-supplied context. Standalone Optimizer callers may supply it.
  std::optional<TrackingContext> tracking;
};

struct Control {
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
};

struct JointModeRequest {
  std::uint64_t id = 0;
  DriveMode mode = DriveMode::DualAckermann;
  // Frozen mechanical positions, including the intended Crab entry direction.
  std::array<double, 4> steering_targets{};
  Twist2d entry_velocity; // Frozen body intent used by the chassis interface.
};

struct ModeFeedback {
  DriveMode actual_mode = DriveMode::DualAckermann;
  bool confirmed = true;
  bool fault = false;
  std::uint64_t request_id = 0;
  double time_in_mode_s = 0.0;
};

enum class FailureReason {
  None,
  InvalidInput,
  NonmonotonicTime,
  InvalidPath,
  FeedbackFault,
  NoFeasiblePlan,
  ModelFailure,
  TransitionFault,
  UnsafeStoppingTrajectory,
  InconsistentFeedback,
  WorkloadExceeded,
  ComputeTimeout
};
enum class ControlPolicy { Stopped, Tracking, Alignment, Capture, ModeTransition, Fault, Blocked };
struct PlanningStats {
  std::size_t branches = 0;
  std::size_t evaluated_rollouts = 0;
  std::size_t feasible_rollouts = 0;
  std::size_t fallback_updates = 0;
  bool budget_exhausted = false;
};

struct PlanningDiagnostics {
  TransitionPhase phase = TransitionPhase::Stable;
  double selected_cost = 0.0;
  double keep_cost = 0.0;
  std::size_t feasible_rollouts = 0;
  // All optimized branches, including nominal and weighted evaluations.
  PlanningStats planning_stats;
  // Number of bounded reduced-intent stopping validations in this compute call.
  std::size_t safety_reductions = 0;
  ControlPolicy control_policy = ControlPolicy::Fault;
  FailureReason failure_reason = FailureReason::None;
  NavigationStatus navigation_status = NavigationStatus::Fault;
  bool goal_reached = false;
  bool stalled = false;
  double path_progress_m = 0.0;
  double remaining_path_m = 0.0;
  double cross_track_error_m = 0.0;
  double goal_distance_m = 0.0;
  double goal_yaw_error_rad = 0.0;
};

// Joint-level execution/model representation. Controller never returns this type.
struct JointCommand : PlanningDiagnostics {
  Action action = Action::SafeStop;
  DriveMode requested_mode = DriveMode::DualAckermann;
  // Drive: endpoint forward kinematics; joints interpolate over the full tick.
  Twist2d body_command;
  Twist2d velocity_intent; // Nominal target, not the FK endpoint after rate limiting.
  std::array<double, 4> steering_targets{};
  std::array<double, 4> wheel_speed_targets{};
  // Present on RequestMode only; retries preserve every field.
  std::optional<JointModeRequest> mode_request;
};

// Body-frame entry intent specifies alignment geometry; it never authorizes Drive.
struct ModeRequest {
  std::uint64_t id = 0;
  DriveMode mode = DriveMode::DualAckermann;
  Twist2d entry_velocity;
};
struct ChassisCommand {
  DriveMode mode = DriveMode::DualAckermann;
  Twist2d target_velocity; // Body-frame target: m/s, m/s, rad/s.
  std::optional<ModeRequest> mode_request;
};
struct Output : PlanningDiagnostics {
  // A valid zero target requests a nominal stop and retains the mode. Absence
  // means no authorized command: the consumer must stop/latch, never reuse Drive.
  std::optional<ChassisCommand> command;
};

} // namespace swerve_mppi
