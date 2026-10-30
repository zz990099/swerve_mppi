#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <swerve_chassis_core/types.hpp>
#include <vector>

namespace swerve_mppi
{
struct Pose
{
  double x = 0, y = 0, yaw = 0;
};
struct Velocity
{
  double vx = 0, vy = 0, wz = 0;
};
struct State
{
  Pose pose;
  Velocity velocity;
  Velocity requested;  // Last request accepted by the executor, distinct from measured velocity.
  std::array<double, 4> steering{};
  std::array<double, 4> wheel_speeds{};  // Angular rad/s, FL/FR/RL/RR.
  swerve_chassis_core::State generator;  // Complete execution snapshot, targets in rad/rad/s.
};
// Borrowed immutable bytes, valid for the complete compute call. Cost: 0..254,
// unknown: 255. Axis-aligned origin and all path/state coordinates share a frame.
struct GridView
{
  const std::uint8_t * data = nullptr;
  std::size_t width = 0, height = 0;
  double resolution = 0, origin_x = 0, origin_y = 0;
  bool valid() const;
  bool world_to_map(double x, double y, std::size_t & mx, std::size_t & my) const;
};
enum class ReferenceKind
{
  ExternalPath,
  GeometricPath,
  FixedHeading,
  WheelAlignment,
  Departure
};
struct MotionReference
{
  ReferenceKind reference_kind = ReferenceKind::GeometricPath;
  double goal_heading_weight = 0;
  double remaining_arc = -1;
  std::vector<Pose> path;
  Pose goal;
  bool goal_active = true;
  std::optional<Velocity> alignment;
  bool precise_heading = false;
  std::optional<Velocity> departure;
};
struct Input
{
  ReferenceKind reference_kind = ReferenceKind::ExternalPath;
  double remaining_arc = -1;        // Measured ordered remaining distance, before local cropping.
  double goal_heading_weight = -1;  // Prepared once per compute, shared by all samples.
  State state;
  std::vector<Pose> path;  // Caller-prepared local ordered section; exclude later return legs.
  Pose goal;               // Full path endpoint, not the local crop endpoint.
  GridView map;
  double dt = 0.05;
  bool goal_active = true;  // Caller enables terminal attraction on the final ordered segment.
  std::optional<Velocity>
    alignment;  // Stopped wheel alignment, still rolled out and collision checked.
  bool precise_heading = false;
  std::optional<Velocity> departure;
};
struct Command
{
  Velocity velocity;
};
enum class Status
{
  Ok,
  InvalidInput,
  NoSolution
};
enum class Rejection
{
  None,
  InvalidState,
  InvalidControl,
  ActuatorResidual,
  GeneratorError,
  Critic,
  Collision
};
struct Diagnostics
{
  std::size_t invalid_input = 0;
  std::size_t controller_errors = 0;
  std::size_t invalid_state = 0, invalid_control = 0, actuator_residual = 0;
  std::size_t generator_error = 0, critic_rejected = 0, collision_rejected = 0, critic_errors = 0;
};
struct TrackingCost
{
  double path = 0, heading = 0, velocity = 0, alignment = 0, effort = 0, progress = 0, goal = 0;
};
struct CandidateDiagnostic
{
  double total = 0;
  bool valid = false;
  TrackingCost tracking;
};
struct Result
{
  Command command;
  Status status = Status::InvalidInput;
  double cost = 0, compute_ms = 0, effective_samples = 0;
  std::size_t evaluated = 0;
  Diagnostics diagnostics;
  bool braking_selected = false;
  bool presteering = false;
  double reference_speed = 0;
  double noise_scale = 0, branch_distance = -1;
  double goal_heading_weight = 0;
  std::uint32_t reference_limit = 0;
  bool reorientation_active = false;
  bool completed_transition = false;
  int branch_phase =
    0;  // 0 tracking, 1 body alignment, 2 wheel alignment, 3 departure, 4 heading settle.
  std::optional<MotionReference> motion_reference;
  // mean, fresh, brake, slow, recovery and selected. Total also includes custom critics.
  std::array<CandidateDiagnostic, 6> candidates{};
  std::vector<Pose> prediction;
  std::vector<Velocity> controls;
};
struct Trajectory
{
  std::vector<State> states;  // Includes initial state.
  std::vector<Velocity> controls;
  double actuator_cost = 0, residual_cost = 0;
  bool valid = true;
  Rejection rejection = Rejection::None;
};
double wrap_angle(double value);
double distance(const Pose & a, const Pose & b);
}  // namespace swerve_mppi
