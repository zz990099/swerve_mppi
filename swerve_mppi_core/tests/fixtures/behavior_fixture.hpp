#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "chassis_fixture.hpp"
namespace swerve_mppi::test
{
inline void check(bool ok, const char * message)
{
  if (!ok) {
    throw std::runtime_error(message);
  }
}
inline double measured_clearance(
  const Pose2d & from, const Pose2d & to, const std::vector<CircleObstacle> & obstacles,
  const Config & c)
{
  double minimum = std::numeric_limits<double>::infinity();
  const double dx = to.x - from.x, dy = to.y - from.y;
  const double length2 = dx * dx + dy * dy;
  for (const auto & obstacle : obstacles) {
    const double t =
      length2 > 1e-12
        ? std::clamp(((obstacle.x - from.x) * dx + (obstacle.y - from.y) * dy) / length2, 0.0, 1.0)
        : 0.0;
    minimum = std::min(
      minimum, std::hypot(from.x + t * dx - obstacle.x, from.y + t * dy - obstacle.y) -
                 obstacle.radius - c.robot_radius_m - c.collision_margin_m);
  }
  return minimum;
}
// Independent encoder fixture. It does not call DriveModel, TransitionModel,
// Kinematics::forward or a controller prediction to advance its state.
inline void actuate(VehicleState & s, const PlantTargets & command, const Config & c)
{
  const double x[] = {c.wheelbase_m / 2, c.wheelbase_m / 2, -c.wheelbase_m / 2, -c.wheelbase_m / 2};
  const double y[] = {c.track_m / 2, -c.track_m / 2, c.track_m / 2, -c.track_m / 2};
  auto encoded = [&]() {
    Twist2d v;
    double moment = 0;
    for (std::size_t i = 0; i < 4; ++i) {
      const double vx = s.wheel_speeds[i] * std::cos(s.steering_angles[i]);
      const double vy = s.wheel_speeds[i] * std::sin(s.steering_angles[i]);
      v.vx += vx / 4;
      v.vy += vy / 4;
      v.wz += x[i] * vy - y[i] * vx;
      moment += x[i] * x[i] + y[i] * y[i];
    }
    v.wz /= moment;
    return v;
  };
  auto sample = [&](const JointSample & target) {
    const auto v = encoded();
    // Independent midpoint integration converges to sample-and-hold SE(2).
    constexpr int subdivisions = 64;
    const double h = target.model_period_s / subdivisions;
    for (int j = 0; j < subdivisions; ++j) {
      const double yaw = s.pose.yaw + v.wz * h / 2;
      s.pose.x += (std::cos(yaw) * v.vx - std::sin(yaw) * v.vy) * h;
      s.pose.y += (std::sin(yaw) * v.vx + std::cos(yaw) * v.vy) * h;
      s.pose.yaw = std::atan2(std::sin(s.pose.yaw + v.wz * h), std::cos(s.pose.yaw + v.wz * h));
    }
    s.steering_angles = target.angles;
    s.wheel_speeds = target.speeds;
    for (std::size_t i = 0; i < 4; ++i) {
      check(
        std::abs(s.steering_angles[i]) <= c.steering_limit_rad + 1e-9, "steering stop violated");
      check(std::abs(s.wheel_speeds[i]) <= c.max_wheel_speed_mps + 1e-9, "wheel speed violated");
    }
  };
  if (command.samples.empty())
    sample({command.steering_targets, command.wheel_speed_targets, c.model_period_s});
  else
    for (const auto & target : command.samples) sample(target);
  s.velocity = encoded();
  s.actual_mode = command.feedback.actual_mode;
  s.mode_confirmed = command.feedback.confirmed;
  s.mode_fault = command.feedback.fault;
  s.mode_request_id = command.feedback.request_id;
  s.accepted_mode_request = command.feedback.accepted_mode_request;
  s.time_in_mode_s = command.feedback.time_in_mode_s;
  s.stamp_ns = *add_duration(s.stamp_ns, c.model_period_s);
}
inline void advance_input(ControllerInput & input, const Output * output = nullptr)
{
  input.planning_stamp_ns = input.vehicle.stamp_ns;
  if (output && output->command_id != 0) {
    input.previous_command_application = CommandApplication{
      output->command_id, output->computed_stamp_ns, output->computed_stamp_ns,
      output->computed_stamp_ns};
  } else {
    input.previous_command_application.reset();
  }
}
inline ControllerInput scenario_input(const std::string & scenario)
{
  ControllerInput input;
  input.vehicle.stamp_ns = *duration_nanoseconds(1.0);
  input.planning_stamp_ns = input.vehicle.stamp_ns;
  input.vehicle.time_in_mode_s = 2;
  if (scenario == "straight") {
    input.reference_path = {{0, 0, 0}, {1, 0, 0}};
  } else if (scenario == "lateral") {
    input.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
  } else if (scenario == "spin") {
    input.reference_path = {{0, 0, 0}, {0, 0, 1.8}};
  } else if (scenario == "spin_translation") {
    input.vehicle.actual_mode = DriveMode::Spin;
    input.vehicle.pose = {.93, .21, .49};
    const double angle = std::atan2(.6, .5);
    input.vehicle.steering_angles = {-angle, angle, angle, -angle};
    input.reference_path = {{.93, .21, .49}, {1.43, .21, .89}};
    input.heading_policy = PathHeadingPolicy::GoalOnly;
    for (double a : {.18, .42}) {
      for (double offset : {-.9, .9}) {
        input.obstacles.push_back(
          {(2 + offset) * std::sin(a), 2 - (2 + offset) * std::cos(a), .08});
      }
    }
  } else if (scenario == "cusp") {
    input.reference_path = {{0, 0, 0}, {1, 0, 0}, {0, 0, 0}};
  } else if (scenario == "loop") {
    input.reference_path = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0}};
  } else if (scenario == "short_cusp") {
    input.reference_path = {{0, 0, 0}, {.2, 0, 0}, {0, 0, 0}};
  } else if (scenario == "short_loop") {
    input.reference_path = {{0, 0, 0}, {.1, 0, 0}, {.1, .1, 0}, {0, .1, 0}, {0, 0, 0}};
  } else if (scenario == "short_corner") {
    input.reference_path = {{0, 0, 0}, {.2, 0, 0}, {.2, .08, 0}, {.12, .08, 0}};
    input.vehicle.pose = {.12, 0, 0};
  } else if (scenario == "reverse") {
    input.reference_path = {{0, 0, 0}, {-1, 0, 0}};
  } else if (scenario == "final_yaw") {
    input.reference_path = {{0, 0, 0}, {1, 0, 1.2}};
    input.heading_policy = PathHeadingPolicy::GoalOnly;
  } else if (scenario == "scurve" || scenario == "scurve_duplicates") {
    for (int i = 0; i <= 60; ++i) {
      const double x = 3.0 * i / 60;
      input.reference_path.push_back(
        {x, .35 * std::sin(2 * std::acos(-1.0) * x / 3),
         std::atan(.35 * 2 * std::acos(-1.0) / 3 * std::cos(2 * std::acos(-1.0) * x / 3))});
      if (scenario == "scurve_duplicates") {
        input.reference_path.push_back(input.reference_path.back());
      }
    }
  } else if (scenario == "curve" || scenario == "near_obstacles") {
    for (int i = 0; i <= 40; ++i) {
      double a = .7 * i / 40;
      input.reference_path.push_back({2 * std::sin(a), 2 * (1 - std::cos(a)), a});
    }
    if (scenario == "near_obstacles") {
      for (int i = 0; i < 20; ++i) {
        const double a = .7 * i / 19;
        for (double offset : {-.75, .75}) {
          input.obstacles.push_back(
            {(2 + offset) * std::sin(a), 2 - (2 + offset) * std::cos(a), .05});
        }
      }
    }
  } else {
    throw std::runtime_error("unknown scenario");
  }
  return input;
}
}  // namespace swerve_mppi::test
