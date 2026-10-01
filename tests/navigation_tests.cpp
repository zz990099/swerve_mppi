#include "swerve_mppi/controller.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace swerve_mppi;
namespace {
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
void test_path_progress_and_replan() {
  Config c;
  PathManager manager(c);
  ControllerInput in;
  in.reference_path = {{0, 0, 0}, {2, 0, 0}, {2, 2, 1.57}};
  auto p = manager.update(in);
  check(p.changed && p.remaining_m == 4 && p.local_path.back().x == c.path_lookahead_m,
        "local target must follow arc length rather than the remote goal");
  in.vehicle.pose.x = .5;
  p = manager.update(in);
  check(!p.changed && std::abs(p.progress_m - .5) < 1e-9 && p.local_path.front().x == .5,
        "progress must prune the already traversed path");
  in.vehicle.pose.x = .3;
  p = manager.update(in);
  check(std::abs(p.progress_m - .5) < 1e-9,
        "progress must not regress after backward localization noise");
  in.reference_path = {{.3, 0, 0}, {-1, 0, 0}};
  p = manager.update(in);
  check(p.changed && p.progress_m == 0 && p.local_path.back().x < 0,
        "replanning must reset progress and preserve reverse waypoint order");
  ++in.path_id;
  check(manager.update(in).changed, "new task ID must restart an identical path");
}
void test_loops_duplicates_and_yaw() {
  Config c;
  PathManager manager(c);
  ControllerInput in;
  in.reference_path = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {0, 0, 0}, {2, -2, 0}};
  auto p = manager.update(in);
  check(p.progress_m == 0 && p.remaining_m > 10,
        "crossing must not jump to a later identical pose");
  GoalManager goal(c);
  in.reference_path.pop_back();
  p = manager.update(in);
  check(!goal.update(in.vehicle, p, false).position_acquired,
        "a closed path must not complete merely because its goal equals its start");
  in.reference_path = {{0, 0, 3.1}, {0, 0, 3.1}, {2, 0, -3.1}};
  p = manager.update(in);
  check(std::abs(std::abs(p.local_path.back().yaw) - std::acos(-1.0)) < 1e-6,
        "interpolated body yaw must use the short angular distance");
  in.reference_path = {{0, 0, 0}, {0, 0, 1}};
  p = manager.update(in);
  check(p.local_path.size() == 1 && p.goal.yaw == 1 && p.remaining_m == 0,
        "zero-length paths must retain the final yaw goal");
}
void test_cusp_target() {
  Config c;
  PathManager manager(c);
  ControllerInput in;
  in.reference_path = {{0, 0, 0}, {1, 0, 0}, {1, 0, 0}, {0, 0, 0}};
  manager.update(in);
  in.vehicle.pose.x = .5;
  auto p = manager.update(in);
  check(p.local_path.back().x == 1 && !p.terminal,
        "lookahead must not reverse before reaching a path cusp");
  in.vehicle.pose.x = .97;
  p = manager.update(in);
  check(p.progress_m >= 1 && p.local_path.back().x < .97,
        "captured cusp must advance to the reverse path segment");
}
void test_completion_requires_measured_stop() {
  Config c;
  GoalManager manager(c);
  PathReference path;
  VehicleState state;
  state.stamp_s = 1;
  state.wheel_speeds.fill(.1);
  check(!manager.update(state, path, false).complete,
        "zero body odometry cannot hide moving wheels");
  state.stamp_s = 4.1;
  check(manager.update(state, path, false).stalled,
        "persistent wheel motion at the goal must expose a stall");
  state.wheel_speeds.fill(0);
  state.stamp_s = 5;
  check(!manager.update(state, path, false).complete,
        "first stopped sample cannot complete the dwell");
  state.stamp_s = 5.2;
  state.mode_confirmed = false;
  check(!manager.update(state, path, true).complete, "pending execution must reset settling");
  state.mode_confirmed = true;
  state.stamp_s = 6;
  check(!manager.update(state, path, false).complete, "confirmation must begin a fresh dwell");
  state.stamp_s = 6.31;
  check(manager.update(state, path, false).complete,
        "measured stopped dwell must complete the goal");
  manager.reset();
  state.pose.x = .08;
  check(!manager.update(state, path, false).position_acquired,
        "outside tolerance must require translation");
}
void test_replan_execution_boundaries() {
  Config c;
  c.noise_v_mps = c.noise_w_radps = 0;
  Controller controller(c);
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 1}};
  const auto request = controller.compute(in);
  check(request.action == Action::RequestMode && request.mode_request.has_value(),
        "terminal yaw must request a measured Spin transition");
  in.vehicle.stamp_s += c.dt_s;
  in.reference_path = {{0, 0, 0}, {1, 0, 0}};
  const auto retry = controller.compute(in);
  check(retry.action == Action::RequestMode && retry.mode_request &&
            retry.mode_request->id == request.mode_request->id &&
            retry.mode_request->steering_targets == request.mode_request->steering_targets,
        "replan must not mutate an already issued execution request");
  in.vehicle.stamp_s += c.confirmation_timeout_s;
  check(controller.compute(in).action == Action::SafeStop,
        "replan must not extend a pending request deadline");
  controller.reset();
  in.vehicle.stamp_s += c.dt_s;
  in.vehicle.actual_mode = DriveMode::Crab;
  in.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
  check(controller.compute(in).action == Action::Hold, "lateral path must begin local alignment");
  in.vehicle.stamp_s += c.dt_s;
  in.reference_path = {{0, 0, 0}, {1, 0, 0}};
  const auto replanned = controller.compute(in);
  check(replanned.action == Action::Drive && replanned.body_command.vx > 0 &&
            replanned.body_command.vy == 0,
        "new geometry must clear the obsolete same-mode alignment intent");
}
void test_task_restart_and_stall() {
  Config c;
  Controller controller(c);
  ControllerInput input;
  input.vehicle.stamp_s = 1;
  input.reference_path = {{0, 0, 0}};
  check(!controller.compute(input).goal_reached, "new task must settle before completion");
  input.vehicle.stamp_s += .4;
  check(controller.compute(input).goal_reached, "settled zero-length task must complete");
  input.vehicle.stamp_s += .1;
  ++input.path_id;
  check(!controller.compute(input).goal_reached,
        "same geometry with a new ID must reset completion");
  GoalManager manager(c);
  PathReference path;
  path.goal = {1, 0, 0};
  path.remaining_m = 1;
  VehicleState state;
  state.stamp_s = 1;
  check(!manager.update(state, path, false).stalled, "new path must not report a stale stall");
  state.stamp_s = 4.1;
  check(manager.update(state, path, false).stalled,
        "no progress must expose a bounded-time diagnostic");
  state.pose.x = .1;
  state.stamp_s += .1;
  path.progress_m = .1;
  check(!manager.update(state, path, false).stalled,
        "measured progress must clear the stall diagnostic");
}
} // namespace
int main() {
  try {
    test_path_progress_and_replan();
    test_loops_duplicates_and_yaw();
    test_completion_requires_measured_stop();
    test_task_restart_and_stall();
    test_cusp_target();
    test_replan_execution_boundaries();
    std::cout << "Navigation regressions passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
