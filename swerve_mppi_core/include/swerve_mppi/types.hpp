#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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
struct Input
{
  State state;
  std::vector<Pose> path;  // Caller-prepared local ordered section; exclude later return legs.
  Pose goal;               // Full path endpoint, not the local crop endpoint.
  GridView map;
  double dt = 0.05;
  bool goal_active = true;  // Caller enables terminal attraction on the final ordered segment.
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
struct Result
{
  Command command;
  Status status = Status::InvalidInput;
  double cost = 0, compute_ms = 0, effective_samples = 0;
  std::size_t evaluated = 0;
  std::vector<Pose> prediction;
};
struct Trajectory
{
  std::vector<State> states;  // Includes initial state.
  std::vector<Velocity> controls;
  double actuator_cost = 0, residual_cost = 0;
  bool valid = true;
};
double wrap_angle(double value);
double distance(const Pose & a, const Pose & b);
}  // namespace swerve_mppi
