#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/executor.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace swerve_mppi;
namespace {
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
// Independent encoder fixture. It does not call DriveModel, TransitionModel,
// Kinematics::forward or a controller prediction to advance its state.
void actuate(VehicleState &s, const ExecutionResult &command, const Config &c) {
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
void run(const std::string &scenario, unsigned seed) {
  Config c;
  c.random_seed = seed;
  Controller controller(c);
  ModeExecutor executor(c);
  ControllerInput input;
  input.vehicle.stamp_s = 1;
  input.vehicle.time_in_mode_s = 2;
  if (scenario == "straight")
    input.reference_path = {{0, 0, 0}, {1, 0, 0}};
  else if (scenario == "lateral")
    input.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
  else if (scenario == "spin")
    input.reference_path = {{0, 0, 0}, {0, 0, 1.8}};
  else if (scenario == "curve") {
    for (int i = 0; i <= 40; ++i) {
      double a = .7 * i / 40;
      input.reference_path.push_back({2 * std::sin(a), 2 * (1 - std::cos(a)), a});
    }
  } else
    throw std::runtime_error("unknown scenario");
  const auto goal = input.reference_path.back();
  int hold = 0, stalled = 0, max_stalled = 0, switches = 0;
  for (int tick = 0; tick < 120; ++tick) {
    auto previous_mode = input.vehicle.actual_mode;
    const auto command = controller.compute(input);
    hold += command.action == Action::Hold;
    check(command.action != Action::SafeStop, "controller unexpectedly stopped");
    const auto result = executor.update(command, input.vehicle);
    check(!result.feedback.fault, "execution fault in default noisy closed loop");
    if (command.action == Action::Drive)
      check(input.vehicle.mode_confirmed, "drive preceded actual mode confirmation");
    actuate(input.vehicle, result, c);
    switches += input.vehicle.actual_mode != previous_mode;
    const double distance =
        std::hypot(goal.x - input.vehicle.pose.x, goal.y - input.vehicle.pose.y);
    const double yaw_error = std::abs(angle_distance(goal.yaw, input.vehicle.pose.yaw));
    const bool unfinished = scenario == "spin" ? yaw_error > .3 : distance > .3;
    const bool stationary =
        std::hypot(input.vehicle.velocity.vx, input.vehicle.velocity.vy) < .02 &&
        std::abs(input.vehicle.velocity.wz) < .02;
    stalled = unfinished && stationary ? stalled + 1 : 0;
    max_stalled = std::max(max_stalled, stalled);
  }
  const double distance = std::hypot(goal.x - input.vehicle.pose.x, goal.y - input.vehicle.pose.y);
  const double yaw_error = std::abs(angle_distance(goal.yaw, input.vehicle.pose.yaw));
  std::cout << scenario << " seed=" << seed << " distance=" << distance << " yaw=" << yaw_error
            << " hold=" << hold << " stall_ticks=" << max_stalled << " switches=" << switches
            << std::endl;
  check(distance < .2, "insufficient translation progress with default noise");
  check(yaw_error < .35, "insufficient heading progress with default noise");
  check(max_stalled < 30, "unfinished task stalled for three seconds");
  check(switches <= (scenario == "lateral" || scenario == "spin" ? 1 : 0),
        "unexpected discrete mode changes");
}
} // namespace
int main(int argc, char **argv) {
  try {
    check(argc == 2, "a scenario is required");
    for (unsigned seed : {1u, 7u, 42u, 73u, 101u})
      run(argv[1], seed);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
