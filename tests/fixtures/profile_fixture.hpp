#pragma once
#include "behavior_fixture.hpp"
#include "swerve_mppi/execution/profile_runner.hpp"

namespace swerve_mppi::test
{
// Ideal encoder plant driven ONLY by ProfileRunner's public joint samples.
// No ExecutionResult target, ActuationPlan endpoint, production FK or model is
// used for integration. Midpoint quadrature approximates ideal servo tracking;
// this is not Gazebo contact/friction calibration.
inline double actuate_profile(
  VehicleState & state, ProfileRunner & runner, const ModeFeedback & mode, const Config & c,
  double wall_start, const std::vector<CircleObstacle> & obstacles)
{
  const double start = state.stamp_s;
  double clearance = measured_clearance(state.pose, state.pose, obstacles, c);
  const double x[] = {c.wheelbase_m / 2, c.wheelbase_m / 2, -c.wheelbase_m / 2, -c.wheelbase_m / 2};
  const double y[] = {c.track_m / 2, -c.track_m / 2, c.track_m / 2, -c.track_m / 2};
  auto measured_twist = [&](const JointTargets & joints) {
    Twist2d v;
    double moment = 0;
    for (std::size_t i = 0; i < 4; ++i) {
      const double speed = joints.wheel_angular_speeds[i] * c.wheel_radius_m;
      const double vx = speed * std::cos(joints.steering_angles[i]);
      const double vy = speed * std::sin(joints.steering_angles[i]);
      v.vx += vx / 4;
      v.vy += vy / 4;
      v.wz += x[i] * vy - y[i] * vx;
      moment += x[i] * x[i] + y[i] * y[i];
    }
    v.wz /= moment;
    return v;
  };
  constexpr int substeps = 10;  // 100 Hz actuator intervals, 200 Hz samples.
  const double h = c.dt_s / substeps;
  auto previous = runner.sample(start, wall_start);
  check(previous.has_value(), "profile start sample unavailable");
  for (int step = 0; step < substeps; ++step) {
    const double middle = (step + .5) * h, end = (step + 1) * h;
    const auto targets = runner.sample(start + middle, wall_start + middle);
    check(targets.has_value(), "high-rate profile sample unavailable");
    const auto velocity = measured_twist(*targets);
    const auto from = state.pose;
    const double yaw = state.pose.yaw + velocity.wz * h / 2;
    state.pose.x += (std::cos(yaw) * velocity.vx - std::sin(yaw) * velocity.vy) * h;
    state.pose.y += (std::sin(yaw) * velocity.vx + std::cos(yaw) * velocity.vy) * h;
    state.pose.yaw = wrap_angle(state.pose.yaw + velocity.wz * h);
    clearance = std::min(clearance, measured_clearance(from, state.pose, obstacles, c));
    const auto endpoint = runner.sample(start + end, wall_start + end);
    check(endpoint.has_value(), "profile interval endpoint unavailable");
    for (std::size_t i = 0; i < 4; ++i) {
      check(
        std::abs(endpoint->steering_angles[i]) <= c.steering_limit_rad + 1e-9 &&
          std::abs(endpoint->wheel_angular_speeds[i] * c.wheel_radius_m) <=
            c.max_wheel_speed_mps + 1e-9,
        "profile sample exceeds joint limits");
      check(
        std::abs(endpoint->steering_angles[i] - previous->steering_angles[i]) <=
            c.max_steer_rate_radps * h + 1e-9 &&
          std::abs(endpoint->wheel_angular_speeds[i] - previous->wheel_angular_speeds[i]) *
              c.wheel_radius_m <=
            c.max_wheel_accel_mps2 * h + 1e-9,
        "sampled profile exceeds joint rate limits");
      state.steering_angles[i] = endpoint->steering_angles[i];
      state.wheel_speeds[i] = endpoint->wheel_angular_speeds[i] * c.wheel_radius_m;
    }
    state.velocity = measured_twist(*endpoint);
    previous = endpoint;
  }
  state.actual_mode = mode.actual_mode;
  state.mode_confirmed = mode.confirmed;
  state.mode_fault = mode.fault;
  state.mode_request_id = mode.request_id;
  state.time_in_mode_s = mode.time_in_mode_s;
  state.stamp_s = start + c.dt_s;
  return clearance;
}
}  // namespace swerve_mppi::test
