#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace swerve_mppi
{
enum class Mode : std::uint8_t
{
  Ackermann = 0,
  Spin = 1,
  Crab = 2
};
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
  std::array<double, 4> steering{};
  std::array<double, 4> wheel_speeds{};  // Rolling m/s, FL/FR/RL/RR.
  Mode mode = Mode::Ackermann;
  Mode requested_mode = Mode::Ackermann;
  bool switching = false;
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
  Mode mode = Mode::Ackermann;
};
enum class Status
{
  Ok,
  Switching,
  InvalidInput,
  NoSolution
};
struct Result
{
  Command command;
  Status status = Status::InvalidInput;
  double cost = 0, compute_ms = 0;
  std::size_t evaluated = 0;
  std::vector<Pose> prediction;
};
struct Trajectory
{
  std::vector<State> states;  // Includes initial state.
  std::vector<Velocity> controls;
  Mode mode = Mode::Ackermann;
  bool valid = true;
};
double wrap_angle(double value);
double distance(const Pose & a, const Pose & b);
bool valid_mode(Mode mode);
}  // namespace swerve_mppi
