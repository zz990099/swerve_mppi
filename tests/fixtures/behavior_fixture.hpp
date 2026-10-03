#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "swerve_mppi/execution/executor.hpp"
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
inline void actuate(VehicleState & s, const ExecutionResult & command, const Config & c)
{
  const bool moving_steering = command.action == Action::Drive;
  const bool stopped = std::hypot(s.velocity.vx, s.velocity.vy) <= c.stopped_linear_mps &&
                       std::abs(s.velocity.wz) <= c.stopped_angular_radps &&
                       std::all_of(s.wheel_speeds.begin(), s.wheel_speeds.end(), [&](double v) {
                         return std::abs(v) <= c.stopped_wheel_speed_mps;
                       });
  const double x[] = {c.wheelbase_m / 2, c.wheelbase_m / 2, -c.wheelbase_m / 2, -c.wheelbase_m / 2};
  const double y[] = {c.track_m / 2, -c.track_m / 2, c.track_m / 2, -c.track_m / 2};
  const auto before = s;
  std::array<double, 4> end_angles = s.steering_angles, end_speeds = s.wheel_speeds;
  double braking_time = std::max(
    std::hypot(s.velocity.vx, s.velocity.vy) / c.max_linear_decel_mps2,
    std::abs(s.velocity.wz) / c.max_angular_decel_radps2);
  for (std::size_t i = 0; i < 4; ++i) {
    end_speeds[i] += std::clamp(
      command.wheel_speed_targets[i] - s.wheel_speeds[i], -c.max_wheel_accel_mps2 * c.dt_s,
      c.max_wheel_accel_mps2 * c.dt_s);
    braking_time = std::max(braking_time, std::abs(s.wheel_speeds[i]) / c.max_wheel_accel_mps2);
    check(std::abs(end_angles[i]) <= c.steering_limit_rad + 1e-9, "steering stop violated");
    check(std::abs(end_speeds[i]) <= c.max_wheel_speed_mps + 1e-9, "wheel speed violated");
  }
  const double steering_time = moving_steering ? c.dt_s
                               : stopped       ? std::max(0.0, c.dt_s - braking_time)
                                               : 0;
  for (std::size_t i = 0; i < 4; ++i) {
    end_angles[i] += std::clamp(
      command.steering_targets[i] - before.steering_angles[i],
      -c.max_steer_rate_radps * steering_time, c.max_steer_rate_radps * steering_time);
  }
  // Drive joint targets ramp over the control period. Stopping actions use a
  // proportional braking ramp, then align only during the remaining stopped
  // time. Derive each intermediate body twist independently from the encoder
  // vectors.
  auto twist_at = [&](double t) {
    Twist2d v;
    double moment = 0;
    for (std::size_t i = 0; i < 4; ++i) {
      const double steering_fraction = moving_steering ? t / c.dt_s
                                       : steering_time > 0
                                         ? std::clamp((t - braking_time) / steering_time, 0.0, 1.0)
                                         : 0;
      const double angle =
        before.steering_angles[i] + (end_angles[i] - before.steering_angles[i]) * steering_fraction;
      const double speed =
        moving_steering
          ? before.wheel_speeds[i] + (end_speeds[i] - before.wheel_speeds[i]) * t / c.dt_s
          : before.wheel_speeds[i] * (braking_time > 0 ? std::max(0.0, 1 - t / braking_time) : 0);
      const double vx = speed * std::cos(angle), vy = speed * std::sin(angle);
      v.vx += vx / 4;
      v.vy += vy / 4;
      v.wz += x[i] * vy - y[i] * vx;
      moment += x[i] * x[i] + y[i] * y[i];
    }
    v.wz /= moment;
    return v;
  };
  constexpr int substeps = 64;
  const double h = (moving_steering ? c.dt_s : std::min(c.dt_s, braking_time)) / substeps;
  for (int step = 0; step < substeps; ++step) {
    const auto v = twist_at((step + .5) * h);
    const double yaw = s.pose.yaw + v.wz * h / 2;
    s.pose.x += (std::cos(yaw) * v.vx - std::sin(yaw) * v.vy) * h;
    s.pose.y += (std::sin(yaw) * v.vx + std::cos(yaw) * v.vy) * h;
    s.pose.yaw = wrap_angle(s.pose.yaw + v.wz * h);
  }
  s.steering_angles = end_angles;
  for (std::size_t i = 0; i < 4; ++i) {
    s.wheel_speeds[i] = moving_steering
                          ? end_speeds[i]
                          : before.wheel_speeds[i] *
                              (braking_time > 0 ? std::max(0.0, 1 - c.dt_s / braking_time) : 0);
  }
  s.velocity = twist_at(c.dt_s);
  s.actual_mode = command.feedback.actual_mode;
  s.mode_confirmed = command.feedback.confirmed;
  s.mode_fault = command.feedback.fault;
  s.mode_request_id = command.feedback.request_id;
  s.accepted_mode_request = command.feedback.accepted_mode_request;
  s.time_in_mode_s = command.feedback.time_in_mode_s;
  s.stamp_s += c.dt_s;
}
inline ControllerInput scenario_input(const std::string & scenario)
{
  ControllerInput input;
  input.vehicle.stamp_s = 1;
  input.vehicle.time_in_mode_s = 2;
  if (scenario == "straight") {
    input.reference_path = {{0, 0, 0}, {1, 0, 0}};
  } else if (scenario == "lateral") {
    input.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
  } else if (scenario == "spin") {
    input.reference_path = {{0, 0, 0}, {0, 0, 1.8}};
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
