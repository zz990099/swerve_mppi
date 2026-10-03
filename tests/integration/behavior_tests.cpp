#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "profile_fixture.hpp"
#include "swerve_mppi/execution/chassis_executor.hpp"
#include "swerve_mppi/execution/timing.hpp"
#include "swerve_mppi/feedback/feedback_adapter.hpp"
#include "swerve_mppi/planning/controller.hpp"

using namespace swerve_mppi;
namespace
{
using namespace swerve_mppi::test;
void run(const std::string & scenario, unsigned seed, bool timed, bool profile)
{
  Config c;
  c.compute_budget_ratio = 0;  // Behavior assertions are independent of host speed.
  c.random_seed = seed;
  Controller controller(c);
  ChassisExecutor executor(c);
  TimedExecutor timed_executor(c, 1);
  ProfileRunner runner(c);
  FeedbackAdapter feedback(c);
  double actuator_wall_s = 10;
  auto input = scenario_input(scenario);
  const auto goal = input.reference_path.back();
  int hold = 0, stalled = 0, max_stalled = 0, switches = 0;
  int completed_tick = -1;
  double max_tracking_error = 0;
  double minimum_clearance =
    measured_clearance(input.vehicle.pose, input.vehicle.pose, input.obstacles, c);
  check(minimum_clearance > 0, "scenario must begin outside the inflated obstacle footprint");
  const bool short_path =
    scenario == "short_cusp" || scenario == "short_loop" || scenario == "short_corner";
  std::size_t next_corner = 1;
  for (int tick = 0; tick < 400; ++tick) {
    auto previous_mode = input.vehicle.actual_mode;
    const auto command = controller.compute(input);
    if (tick == 0 && short_path) {
      check(
        command.command && command.command->target_velocity.vx > 0 &&
          std::abs(command.command->target_velocity.vy) < 1e-9,
        "short paths must first drive toward the uncaptured forward corner");
    }
    max_tracking_error = std::max(max_tracking_error, command.cross_track_error_m);
    if (command.goal_reached && completed_tick < 0) {
      if (short_path) {
        check(
          next_corner + 1 >= input.reference_path.size(),
          "short paths cannot complete before measured ordered corner "
          "capture");
      }
      completed_tick = tick;
      check(is_stopped(input.vehicle, c), "completion requires measured stopped joints and body");
      check(
        command.goal_distance_m <= c.goal_position_tolerance_m &&
          std::abs(command.goal_yaw_error_rad) <= c.goal_yaw_tolerance_rad,
        "completion must satisfy both pose tolerances");
    }
    if (completed_tick >= 0) {
      check(
        command.goal_reached && command.command && command.command->target_velocity.vx == 0 &&
          command.command->target_velocity.vy == 0 && command.command->target_velocity.wz == 0,
        "completed task must remain stopped without stochastic resampling");
      if (tick - completed_tick >= 10) {
        break;
      }
    }
    check(command.command.has_value(), "controller unexpectedly stopped");
    ExecutionResult result;
    if (timed || profile) {
      const double now = input.vehicle.stamp_s;
      CommandEnvelope envelope{
        1,          static_cast<std::uint64_t>(tick + 1), now, command, now, now,
        now + .025, CommandTask::capture(input)};
      const auto guarded = timed_executor.update(envelope, input, now);
      check(
        guarded.timing_error == TimingError::None &&
          guarded.safety_error == ExecutionSafetyError::None && guarded.actuation,
        "default behavior must pass execution-time validation without a "
        "fallback");
      result = guarded.execution;
      if (profile) {
        check(runner.install(guarded, now, actuator_wall_s), "profile install in closed loop");
      }
    } else {
      result = executor.update(command, input.vehicle);
    }
    if (result.feedback.fault) {
      std::cerr << "fault scenario=" << scenario << " seed=" << seed << " tick=" << tick
                << " mode=" << static_cast<int>(input.vehicle.actual_mode)
                << " action=" << static_cast<int>(result.action) << '\n';
      for (std::size_t i = 0; i < 4; ++i) {
        std::cerr << input.vehicle.steering_angles[i] << "->" << result.steering_targets[i]
                  << " speed=" << result.wheel_speed_targets[i] << '\n';
      }
    }
    check(!result.feedback.fault, "execution fault in default noisy closed loop");
    hold += result.action == Action::Hold;
    if (result.action == Action::Drive) {
      check(input.vehicle.mode_confirmed, "drive preceded actual mode confirmation");
    }
    const auto from = input.vehicle.pose;
    if (profile) {
      minimum_clearance = std::min(
        minimum_clearance,
        actuate_profile(
          input.vehicle, runner, result.feedback, c, actuator_wall_s, input.obstacles));
    } else {
      actuate(input.vehicle, result, c);
    }
    if (profile) {
      JointObservation encoders;
      encoders.stamp_s = input.vehicle.stamp_s;
      const std::array<std::string, 4> corners{"rr", "rl", "fr", "fl"};
      for (std::size_t i = 0; i < 4; ++i) {
        encoders.names.push_back(corners[i] + "_wheel_joint");
        encoders.names.push_back(corners[i] + "_steering_joint");
        encoders.positions.push_back(0);
        encoders.positions.push_back(input.vehicle.steering_angles[3 - i]);
        encoders.velocities.push_back(input.vehicle.wheel_speeds[3 - i] / c.wheel_radius_m);
        encoders.velocities.push_back(0);
      }
      const auto snapshot = feedback.make(
        encoders, {input.vehicle.pose, encoders.stamp_s}, result.feedback, encoders.stamp_s);
      check(snapshot.state.has_value(), "sampled encoder feedback must pass synchronized adapter");
      input.vehicle = *snapshot.state;
    }
    actuator_wall_s += c.dt_s;
    minimum_clearance =
      std::min(minimum_clearance, measured_clearance(from, input.vehicle.pose, input.obstacles, c));
    check(minimum_clearance > 0, "measured motion must remain outside inflated obstacles");
    if (short_path) {
      const auto & to = input.vehicle.pose;
      const double dx = to.x - from.x, dy = to.y - from.y;
      const double length2 = dx * dx + dy * dy;
      while (next_corner + 1 < input.reference_path.size()) {
        const auto & corner = input.reference_path[next_corner];
        const double t =
          length2 > 1e-12
            ? std::clamp(((corner.x - from.x) * dx + (corner.y - from.y) * dy) / length2, 0.0, 1.0)
            : 0.0;
        if (
          std::hypot(corner.x - from.x - t * dx, corner.y - from.y - t * dy) >
          c.goal_position_tolerance_m + 1e-9) {
          break;
        }
        ++next_corner;
      }
    }
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
            << " min_clearance=" << minimum_clearance << std::endl;
  check(completed_tick >= 0, "controller failed to complete and settle within 40 seconds");
  check(
    distance <= c.goal_position_tolerance_m && yaw_error <= c.goal_yaw_tolerance_rad,
    "settled pose must remain inside goal tolerances");
  check(max_stalled < 30, "unfinished task stalled for three seconds");
  check(max_tracking_error < .3, "path tracking exceeded the regression corridor");
  check(switches <= 4, "excessive mode changes including terminal pose alignment");
  if (scenario == "near_obstacles") {
    check(minimum_clearance < .2, "near-obstacle scenario must exercise the clearance region");
  }
}
}  // namespace
int main(int argc, char ** argv)
{
  try {
    const bool timed = argc > 2 && std::string(argv[argc - 1]) == "--timed";
    const bool profile = argc > 2 && std::string(argv[argc - 1]) == "--profile";
    const int arguments = argc - (timed || profile ? 1 : 0);
    check(
      arguments == 2 || arguments == 3,
      "usage: behavior_tests scenario [seed] [--timed|--profile]");
    if (arguments == 3) {
      run(argv[1], std::stoul(argv[2]), timed, profile);
      return 0;
    }
    for (unsigned seed : {1u, 7u, 42u, 73u, 101u}) {
      run(argv[1], seed, timed, profile);
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
