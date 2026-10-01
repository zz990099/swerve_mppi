#include "behavior_fixture.hpp"
#include "swerve_mppi/controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace swerve_mppi;
namespace {
using namespace swerve_mppi::test;
void run(const std::string &scenario, unsigned seed) {
  Config c;
  c.random_seed = seed;
  Controller controller(c);
  ModeExecutor executor(c);
  auto input = scenario_input(scenario);
  const auto goal = input.reference_path.back();
  int hold = 0, stalled = 0, max_stalled = 0, switches = 0;
  int completed_tick = -1;
  double max_tracking_error = 0;
  for (int tick = 0; tick < 400; ++tick) {
    auto previous_mode = input.vehicle.actual_mode;
    const auto command = controller.compute(input);
    hold += command.action == Action::Hold;
    max_tracking_error = std::max(max_tracking_error, command.cross_track_error_m);
    if (command.goal_reached && completed_tick < 0) {
      completed_tick = tick;
      check(is_stopped(input.vehicle, c), "completion requires measured stopped joints and body");
      check(command.goal_distance_m <= c.goal_position_tolerance_m &&
                std::abs(command.goal_yaw_error_rad) <= c.goal_yaw_tolerance_rad,
            "completion must satisfy both pose tolerances");
    }
    if (completed_tick >= 0) {
      check(command.goal_reached && command.action == Action::Hold,
            "completed task must remain stopped without stochastic resampling");
      if (tick - completed_tick >= 10)
        break;
    }
    check(command.action != Action::SafeStop, "controller unexpectedly stopped");
    const auto result = executor.update(command, input.vehicle);
    if (result.feedback.fault) {
      std::cerr << "fault scenario=" << scenario << " seed=" << seed << " tick=" << tick
                << " mode=" << static_cast<int>(input.vehicle.actual_mode)
                << " action=" << static_cast<int>(command.action)
                << " body=" << command.body_command.vx << "," << command.body_command.vy << ","
                << command.body_command.wz << '\n';
      for (std::size_t i = 0; i < 4; ++i)
        std::cerr << input.vehicle.steering_angles[i] << "->" << command.steering_targets[i]
                  << " speed=" << command.wheel_speed_targets[i] << '\n';
    }
    check(!result.feedback.fault, "execution fault in default noisy closed loop");
    if (command.action == Action::Drive)
      check(input.vehicle.mode_confirmed, "drive preceded actual mode confirmation");
    actuate(input.vehicle, result, c);
    switches += input.vehicle.actual_mode != previous_mode;
    const double distance =
        std::hypot(goal.x - input.vehicle.pose.x, goal.y - input.vehicle.pose.y);
    const double yaw_error = std::abs(angle_distance(goal.yaw, input.vehicle.pose.yaw));
    const bool unfinished =
        !command.goal_reached &&
        (command.remaining_path_m > c.goal_position_tolerance_m ||
         distance > c.goal_position_tolerance_m || yaw_error > c.goal_yaw_tolerance_rad);
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
            << " complete_tick=" << completed_tick << " cross_track=" << max_tracking_error
            << std::endl;
  check(completed_tick >= 0, "controller failed to complete and settle within 40 seconds");
  check(distance <= c.goal_position_tolerance_m && yaw_error <= c.goal_yaw_tolerance_rad,
        "settled pose must remain inside goal tolerances");
  check(max_stalled < 30, "unfinished task stalled for three seconds");
  check(max_tracking_error < .3, "path tracking exceeded the regression corridor");
  check(switches <= 4, "excessive mode changes including terminal pose alignment");
}
} // namespace
int main(int argc, char **argv) {
  try {
    check(argc == 2 || argc == 3, "a scenario and optional seed are required");
    if (argc == 3) {
      run(argv[1], std::stoul(argv[2]));
      return 0;
    }
    for (unsigned seed : {1u, 7u, 42u, 73u, 101u})
      run(argv[1], seed);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
