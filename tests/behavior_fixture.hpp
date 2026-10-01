#pragma once
#include "swerve_mppi/executor.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
namespace swerve_mppi::test {
inline void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
inline double measured_clearance(const Pose2d &from, const Pose2d &to,
                                 const std::vector<CircleObstacle> &obstacles, const Config &c) {
  double minimum = std::numeric_limits<double>::infinity();
  const double dx = to.x - from.x, dy = to.y - from.y;
  const double length2 = dx * dx + dy * dy;
  for (const auto &obstacle : obstacles) {
    const double t =
        length2 > 1e-12
            ? std::clamp(((obstacle.x - from.x) * dx + (obstacle.y - from.y) * dy) / length2, 0.0,
                         1.0)
            : 0.0;
    minimum =
        std::min(minimum, std::hypot(from.x + t * dx - obstacle.x, from.y + t * dy - obstacle.y) -
                              obstacle.radius - c.robot_radius_m - c.collision_margin_m);
  }
  return minimum;
}
// Independent encoder fixture. It does not call DriveModel, TransitionModel,
// Kinematics::forward or a controller prediction to advance its state.
inline void actuate(VehicleState &s, const ExecutionResult &command, const Config &c) {
  const bool moving_steering = command.action == Action::Drive;
  const bool stopped = std::hypot(s.velocity.vx, s.velocity.vy) <= c.stopped_linear_mps &&
                       std::abs(s.velocity.wz) <= c.stopped_angular_radps &&
                       std::all_of(s.wheel_speeds.begin(), s.wheel_speeds.end(), [&](double v) {
                         return std::abs(v) <= c.stopped_wheel_speed_mps;
                       });
  const double x[] = {c.wheelbase_m / 2, c.wheelbase_m / 2, -c.wheelbase_m / 2, -c.wheelbase_m / 2};
  const double y[] = {c.track_m / 2, -c.track_m / 2, c.track_m / 2, -c.track_m / 2};
  s.velocity = {};
  double moment = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    if (stopped || moving_steering)
      s.steering_angles[i] +=
          std::clamp(command.steering_targets[i] - s.steering_angles[i],
                     -c.max_steer_rate_radps * c.dt_s, c.max_steer_rate_radps * c.dt_s);
    s.wheel_speeds[i] +=
        std::clamp(command.wheel_speed_targets[i] - s.wheel_speeds[i],
                   -c.max_wheel_accel_mps2 * c.dt_s, c.max_wheel_accel_mps2 * c.dt_s);
    check(std::abs(s.steering_angles[i]) <= c.steering_limit_rad + 1e-9, "steering stop violated");
    check(std::abs(s.wheel_speeds[i]) <= c.max_wheel_speed_mps + 1e-9, "wheel speed violated");
    const double vx = s.wheel_speeds[i] * std::cos(s.steering_angles[i]);
    const double vy = s.wheel_speeds[i] * std::sin(s.steering_angles[i]);
    s.velocity.vx += vx / 4;
    s.velocity.vy += vy / 4;
    s.velocity.wz += x[i] * vy - y[i] * vx;
    moment += x[i] * x[i] + y[i] * y[i];
  }
  s.velocity.wz /= moment;
  // Midpoint integration intentionally differs from the core SE(2) integrator.
  const double yaw = s.pose.yaw + s.velocity.wz * c.dt_s / 2;
  s.pose.x += (std::cos(yaw) * s.velocity.vx - std::sin(yaw) * s.velocity.vy) * c.dt_s;
  s.pose.y += (std::sin(yaw) * s.velocity.vx + std::cos(yaw) * s.velocity.vy) * c.dt_s;
  s.pose.yaw = wrap_angle(s.pose.yaw + s.velocity.wz * c.dt_s);
  s.actual_mode = command.feedback.actual_mode;
  s.mode_confirmed = command.feedback.confirmed;
  s.mode_fault = command.feedback.fault;
  s.mode_request_id = command.feedback.request_id;
  s.time_in_mode_s = command.feedback.time_in_mode_s;
  s.stamp_s += c.dt_s;
}
inline ControllerInput scenario_input(const std::string &scenario) {
  ControllerInput input;
  input.vehicle.stamp_s = 1;
  input.vehicle.time_in_mode_s = 2;
  if (scenario == "straight")
    input.reference_path = {{0, 0, 0}, {1, 0, 0}};
  else if (scenario == "lateral")
    input.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
  else if (scenario == "spin")
    input.reference_path = {{0, 0, 0}, {0, 0, 1.8}};
  else if (scenario == "cusp")
    input.reference_path = {{0, 0, 0}, {1, 0, 0}, {0, 0, 0}};
  else if (scenario == "loop")
    input.reference_path = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0}};
  else if (scenario == "short_cusp")
    input.reference_path = {{0, 0, 0}, {.2, 0, 0}, {0, 0, 0}};
  else if (scenario == "short_loop")
    input.reference_path = {{0, 0, 0}, {.1, 0, 0}, {.1, .1, 0}, {0, .1, 0}, {0, 0, 0}};
  else if (scenario == "short_corner") {
    input.reference_path = {{0, 0, 0}, {.2, 0, 0}, {.2, .08, 0}, {.12, .08, 0}};
    input.vehicle.pose = {.12, 0, 0};
  } else if (scenario == "reverse")
    input.reference_path = {{0, 0, 0}, {-1, 0, 0}};
  else if (scenario == "final_yaw") {
    input.reference_path = {{0, 0, 0}, {1, 0, 1.2}};
    input.heading_policy = PathHeadingPolicy::GoalOnly;
  } else if (scenario == "scurve") {
    for (int i = 0; i <= 60; ++i) {
      const double x = 3.0 * i / 60;
      input.reference_path.push_back(
          {x, .35 * std::sin(2 * std::acos(-1.0) * x / 3),
           std::atan(.35 * 2 * std::acos(-1.0) / 3 * std::cos(2 * std::acos(-1.0) * x / 3))});
    }
  } else if (scenario == "curve" || scenario == "near_obstacles") {
    for (int i = 0; i <= 40; ++i) {
      double a = .7 * i / 40;
      input.reference_path.push_back({2 * std::sin(a), 2 * (1 - std::cos(a)), a});
    }
    if (scenario == "near_obstacles")
      for (int i = 0; i < 20; ++i) {
        const double a = .7 * i / 19;
        for (double offset : {-.75, .75})
          input.obstacles.push_back(
              {(2 + offset) * std::sin(a), 2 - (2 + offset) * std::cos(a), .05});
      }
  } else
    throw std::runtime_error("unknown scenario");
  return input;
}
} // namespace swerve_mppi::test
