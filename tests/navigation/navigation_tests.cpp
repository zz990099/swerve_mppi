#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "planning/detail/planner.hpp"
#include "swerve_mppi/planning/controller.hpp"
#include "swerve_mppi/planning/critics.hpp"
using namespace swerve_mppi;
namespace
{
void check(bool ok, const char * message)
{
  if (!ok) {
    throw std::runtime_error(message);
  }
}
void test_path_progress_and_replan()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  PathManager manager(c);
  ControllerInput in;
  in.reference_path = {{0, 0, 0}, {2, 0, 0}, {2, 2, 1.57}};
  auto p = manager.update(in);
  check(
    p.changed && p.remaining_m == 4 && p.local_path.back().x == c.path_lookahead_m,
    "local target must follow arc length rather than the remote goal");
  in.vehicle.pose.x = .5;
  p = manager.update(in);
  check(
    !p.changed && std::abs(p.progress_m - .5) < 1e-9 && p.local_path.front().x == .5,
    "progress must prune the already traversed path");
  in.vehicle.pose.x = .3;
  p = manager.update(in);
  check(
    std::abs(p.progress_m - .5) < 1e-9,
    "progress must not regress after backward localization noise");
  in.reference_path = {{.3, 0, 0}, {-1, 0, 0}};
  p = manager.update(in);
  check(
    p.changed && p.progress_m == 0 && p.local_path.back().x < 0,
    "replanning must reset progress and preserve reverse waypoint order");
  ++in.path_id;
  check(manager.update(in).changed, "new task ID must restart an identical path");
}
void test_loops_duplicates_and_yaw()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  PathManager manager(c);
  ControllerInput in;
  in.reference_path = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {0, 0, 0}, {2, -2, 0}};
  auto p = manager.update(in);
  check(
    p.progress_m == 0 && p.remaining_m > 10, "crossing must not jump to a later identical pose");
  GoalManager goal(c);
  in.reference_path.pop_back();
  p = manager.update(in);
  check(
    !goal.update(in.vehicle, p, false).position_acquired,
    "a closed path must not complete merely because its goal equals its "
    "start");
  in.reference_path = {{0, 0, 3.1}, {0, 0, 3.1}, {2, 0, -3.1}};
  p = manager.update(in);
  check(
    std::abs(std::abs(p.local_path.back().yaw) - std::acos(-1.0)) < 1e-6,
    "interpolated body yaw must use the short angular distance");
  in.reference_path = {{0, 0, 0}, {0, 0, 1}};
  p = manager.update(in);
  check(
    p.local_path.size() == 1 && p.goal.yaw == 1 && p.remaining_m == 0,
    "zero-length paths must retain the final yaw goal");
}
void test_cusp_target()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  PathManager manager(c);
  ControllerInput in;
  in.reference_path = {{0, 0, 0}, {1, 0, 0}, {1, 0, 0}, {0, 0, 0}};
  manager.update(in);
  in.vehicle.pose.x = .5;
  auto p = manager.update(in);
  check(
    p.local_path.back().x == 1 && !p.terminal,
    "lookahead must not reverse before reaching a path cusp");
  in.vehicle.pose.x = .97;
  p = manager.update(in);
  check(
    p.progress_m >= 1 && p.local_path.back().x < .97,
    "captured cusp must advance to the reverse path segment");
}
void test_short_paths_preserve_segment_order()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  const std::vector<std::vector<Pose2d>> paths = {
    {{0, 0, 0}, {.2, 0, 0}, {.2, .2, 0}, {0, .2, 0}, {0, 0, 0}},
    {{0, 0, 0}, {.25, .25, 0}, {0, .25, 0}, {.25, 0, 0}, {0, 0, 0}},
    {{0, 0, 0}, {.4, 0, 0}, {.4, .04, 0}, {0, .04, 0}}};
  for (const auto & path : paths) {
    PathManager manager(c);
    detail::Planner controller(c);
    ControllerInput in;
    in.reference_path = path;
    in.vehicle.pose = {.01, .03, 0};
    in.vehicle.time_in_mode_s = 2;
    for (int tick = 0; tick < 8; ++tick) {
      in.vehicle.stamp_s = 1 + tick * c.dt_s;
      const auto reference = manager.update(in);
      check(
        reference.progress_m < .05 && reference.remaining_m > .5,
        "short loops/crossings/foldbacks cannot match a nearby later "
        "segment");
      check(
        !controller.compute(in).goal_reached,
        "a stationary robot near a short path endpoint cannot complete the "
        "task");
    }
  }
  PathManager manager(c);
  ControllerInput in;
  in.reference_path = paths.front();
  in.vehicle.pose = {0, .02, 0};
  manager.update(in);
  for (std::size_t i = 1; i < in.reference_path.size(); ++i) {
    in.vehicle.pose = in.reference_path[i];
    const auto reference = manager.update(in);
    check(
      std::abs(reference.progress_m - .2 * i) < 1e-9,
      "captured short-loop vertices must advance in order");
  }
  check(
    manager.update(in).remaining_m == 0,
    "an actually traversed short loop must reach the terminal segment");
  in.reference_path = {{0, 0, 0}, {.3, 0, 0}, {.3, .3, 0}};
  in.vehicle.pose = {};
  manager.update(in);
  in.vehicle.pose = {.25, .15, 0};
  const auto reference = manager.update(in);
  check(
    reference.progress_m <= .25 + 1e-9 && reference.corner_target &&
      reference.local_path.back().y == 0,
    "projection cannot pass an uncaptured corner even when its next leg is "
    "nearer");
}
void test_dense_segment_capture()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  PathManager manager(c);
  ControllerInput in;
  in.reference_path = {{0, 0, 0},   {0, 0, 0},   {.02, 0, 0}, {.04, 0, 0},
                       {.06, 0, 0}, {.08, 0, 0}, {.1, 0, 0},  {1, 0, 0}};
  manager.update(in);
  in.vehicle.pose = {.12, .15, 0};
  check(
    std::abs(manager.update(in).progress_m - .12) < 1e-9,
    "smooth samples must advance in order despite cross-track error");
  in.reference_path = {{0, 0, 0}, {.02, 0, 0}, {.04, 0, 0}, {.04, .3, 0}};
  in.vehicle.pose = {};
  const auto captured = manager.update(in);
  check(captured.progress_m == .04, "a nearby sharp corner may be captured within tolerance");
  in.vehicle.pose.y = .1;
  check(
    std::abs(manager.update(in).progress_m - .14) < 1e-9,
    "lookahead corner capture must also advance the matched segment state");
}
void test_completion_requires_measured_stop()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  GoalManager manager(c);
  PathReference path;
  path.goal_eligible = true;
  VehicleState state;
  state.stamp_s = 1;
  state.wheel_speeds.fill(.1);
  check(
    !manager.update(state, path, false).complete, "zero body odometry cannot hide moving wheels");
  state.stamp_s = 4.1;
  check(
    manager.update(state, path, false).stalled,
    "persistent wheel motion at the goal must expose a stall");
  state.wheel_speeds.fill(0);
  state.stamp_s = 5;
  check(
    !manager.update(state, path, false).complete, "first stopped sample cannot complete the dwell");
  state.stamp_s = 5.2;
  state.mode_confirmed = false;
  check(!manager.update(state, path, true).complete, "pending execution must reset settling");
  state.mode_confirmed = true;
  state.stamp_s = 6;
  check(!manager.update(state, path, false).complete, "confirmation must begin a fresh dwell");
  state.stamp_s = 6.31;
  check(
    !manager.update(state, path, false).complete,
    "missing measured samples cannot count toward continuous stopped dwell");
  for (int tick = 1; tick <= 3; ++tick) {
    state.stamp_s = 6.31 + tick * c.dt_s;
    manager.update(state, path, false);
  }
  check(
    manager.update(state, path, false).complete, "measured stopped dwell must complete the goal");
  manager.reset();
  state.pose.x = .08;
  check(
    !manager.update(state, path, false).position_acquired,
    "outside tolerance must require translation");
}
void test_effective_target_and_goal_eligibility()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  PathManager manager(c);
  GoalManager goal(c);
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 0}, {.2, 0, 0}, {.2, .08, 0}, {.12, .08, 0}};
  in.vehicle.pose = {.12, 0, 0};
  auto path = manager.update(in);
  check(
    path.target_kind == PathTargetKind::Corner && !path.goal_eligible && path.target.x == .2 &&
      path.target.y == 0 && std::abs(path.target_remaining_m - .08) < 1e-9,
    "an uncaptured corner owns the translation target even near the global "
    "goal");
  // A standalone caller must grant eligibility, not just supply small errors.
  PathReference blocked;
  blocked.goal = in.vehicle.pose;
  check(
    !goal.update(in.vehicle, blocked, false).position_acquired,
    "small goal errors and remaining length cannot override missing "
    "terminal eligibility");
  in.vehicle.pose = {.2, 0, 0};
  in.vehicle.stamp_s += c.dt_s;
  path = manager.update(in);
  check(
    path.target_kind == PathTargetKind::Corner && !path.goal_eligible && path.target.y == .08,
    "capturing one corner cannot grant eligibility through the next corner");
  in.vehicle.pose = {.2, .08, 0};
  in.vehicle.stamp_s += c.dt_s;
  path = manager.update(in);
  check(
    path.target_kind == PathTargetKind::Goal && path.goal_eligible &&
      path.target.x == path.goal.x && path.target.y == path.goal.y,
    "the global goal becomes eligible after all blocking corners are "
    "captured");
}
void test_replan_execution_boundaries()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.noise_v_mps = c.noise_w_radps = 0;
  detail::Planner controller(c);
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 1}};
  const auto request = controller.compute(in);
  check(
    request.action == Action::RequestMode && request.mode_request.has_value(),
    "terminal yaw must request a measured Spin transition");
  in.vehicle.stamp_s += c.dt_s;
  in.reference_path = {{0, 0, 0}, {1, 0, 0}};
  const auto retry = controller.compute(in);
  check(
    retry.action == Action::RequestMode && retry.mode_request &&
      retry.mode_request->id == request.mode_request->id &&
      retry.mode_request->steering_targets == request.mode_request->steering_targets,
    "replan must not mutate an already issued execution request");
  in.vehicle.stamp_s += c.confirmation_timeout_s;
  check(
    controller.compute(in).action == Action::SafeStop,
    "replan must not extend a pending request deadline");
  controller.reset();
  in.vehicle.stamp_s += c.dt_s;
  in.vehicle.actual_mode = DriveMode::Crab;
  in.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
  check(controller.compute(in).action == Action::Hold, "lateral path must begin local alignment");
  in.vehicle.stamp_s += c.dt_s;
  in.reference_path = {{0, 0, 0}, {1, 0, 0}};
  const auto replanned = controller.compute(in);
  check(
    replanned.action == Action::Drive && replanned.body_command.vx > 0 &&
      replanned.body_command.vy == 0,
    "new geometry must clear the obsolete same-mode alignment intent");
}
void test_task_restart_and_stall()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  detail::Planner controller(c);
  ControllerInput input;
  input.vehicle.stamp_s = 1;
  input.reference_path = {{0, 0, 0}};
  check(!controller.compute(input).goal_reached, "new task must settle before completion");
  input.vehicle.stamp_s += .4;
  check(!controller.compute(input).goal_reached, "a sparse stopped sample must restart settling");
  for (int tick = 0; tick < 3; ++tick) {
    input.vehicle.stamp_s += c.dt_s;
    controller.compute(input);
  }
  input.vehicle.stamp_s += c.dt_s;
  check(controller.compute(input).goal_reached, "regular measured stopped dwell must complete");
  input.vehicle.stamp_s += .1;
  ++input.path_id;
  check(
    !controller.compute(input).goal_reached, "same geometry with a new ID must reset completion");
  GoalManager manager(c);
  PathReference path;
  path.goal = {1, 0, 0};
  path.remaining_m = 1;
  VehicleState state;
  state.stamp_s = 1;
  check(!manager.update(state, path, false).stalled, "new path must not report a stale stall");
  state.stamp_s = 4.1;
  check(
    manager.update(state, path, false).stalled,
    "no progress must expose a bounded-time diagnostic");
  state.pose.x = .1;
  state.stamp_s += .1;
  path.progress_m = .1;
  check(
    !manager.update(state, path, false).stalled,
    "measured progress must clear the stall diagnostic");
}
}  // namespace
void test_large_angles_and_derived_errors()
{
  Config c;
  c.compute_budget_ratio = 0;
  const double large = std::numeric_limits<double>::max();
  const double expected = angle_distance(wrap_angle(large), wrap_angle(-large));
  check(
    std::isfinite(angle_distance(large, -large)) && angle_distance(large, -large) == expected &&
      angle_distance(large, large) == 0,
    "finite angles must not overflow before periodic subtraction");
  ControllerInput input;
  input.vehicle.stamp_s = 1;
  input.vehicle.actual_mode = DriveMode::Spin;
  input.vehicle.steering_angles = DriveModel(c).steering_for_mode(DriveMode::Spin, {});
  input.vehicle.pose.yaw = large;
  input.reference_path = {{0, 0, -large}};
  const auto output = Controller(c).compute(input);
  auto normalized = input;
  normalized.vehicle.pose.yaw = wrap_angle(large);
  normalized.reference_path[0].yaw = wrap_angle(-large);
  const auto wrapped = Controller(c).compute(normalized);
  check(
    output.command && wrapped.command && std::isfinite(output.goal_yaw_error_rad) &&
      output.goal_yaw_error_rad == wrapped.goal_yaw_error_rad &&
      output.command->target_velocity.wz == wrapped.command->target_velocity.wz,
    "large finite yaw must behave like its normalized representation");
  auto path_input = input;
  path_input.reference_path = {{0, 0, large}, {2, 0, -large}};
  auto wrapped_path = path_input;
  for (auto & pose : wrapped_path.reference_path) {
    pose.yaw = wrap_angle(pose.yaw);
  }
  const auto path = PathManager(c).update(path_input);
  const auto normal_path = PathManager(c).update(wrapped_path);
  check(
    path.target.yaw == normal_path.target.yaw,
    "path interpolation must reduce large heading before adding local angular progress");
  path_input.tracking = TrackingContext{{}, 2, .5, false, PathHeadingPolicy::FollowPath};
  wrapped_path.tracking = path_input.tracking;
  Trajectory trace;
  trace.valid = true;
  trace.poses = {path_input.vehicle.pose, {1, 0, .2}};
  trace.final_state.pose = trace.poses.back();
  const double path_cost = CriticManager(c).score(path_input, trace);
  check(
    std::isfinite(path_cost) && path_cost == CriticManager(c).score(wrapped_path, trace),
    "path-heading cost must use the same normalized angular interpolation");
  VehicleState state;
  state.pose.yaw = large;
  state.wheel_speeds.fill(.4);
  state.velocity.vx = .4;
  auto normal_state = state;
  normal_state.pose.yaw = wrap_angle(large);
  for (auto control : {Control{.4, 0, 0}, Control{.4, 0, .1}, Control{}}) {
    const auto a = DriveModel(c).step(state, control, c.dt_s);
    const auto b = DriveModel(c).step(normal_state, control, c.dt_s);
    check(
      a.valid && b.valid &&
        std::hypot(a.state.pose.x - b.state.pose.x, a.state.pose.y - b.state.pose.y) < 1e-12 &&
        a.state.pose.yaw == b.state.pose.yaw,
      "fixed/moving steering and braking must normalize heading before integration");
  }
  input = {};
  input.vehicle.pose.x = -large;
  input.reference_path = {{large, 0, 0}};
  const auto invalid = Controller(c).compute(input);
  check(
    !invalid.command && invalid.failure_reason == FailureReason::InvalidPath &&
      std::isfinite(invalid.goal_distance_m) && std::isfinite(invalid.goal_yaw_error_rad),
    "nonfinite derived distance must withhold authorization without leaking NaN diagnostics");
}
void test_path_numerical_boundaries()
{
  Config c;
  c.compute_budget_ratio = 0;
  const double large = std::numeric_limits<double>::max();
  for (const auto & input : std::vector<ControllerInput>{
         {{{1e308, 0, 0}}, {{-1e308, 0, 0}, {0, 0, 0}}},
         {{{0, -1e308, 0}}, {{0, 1e308, 0}, {0, 0, 0}}},
         {{{1e308, 0, 0}}, {{0, 0, 0}, {.25, 0, 0}}},
         {{{large, large, 0}}, {{0, 0, 0}}}}) {
    bool rejected = false;
    try {
      PathManager(c).update(input);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected, "unrepresentable path errors must be rejected by the public path manager");
    const auto out = Controller(c).compute(input);
    check(
      !out.command && out.failure_reason == FailureReason::InvalidPath &&
        out.navigation_status == NavigationStatus::Fault && !out.goal_reached &&
        std::isfinite(out.selected_cost) && std::isfinite(out.keep_cost) &&
        std::isfinite(out.path_progress_m) && std::isfinite(out.remaining_path_m) &&
        std::isfinite(out.cross_track_error_m) && std::isfinite(out.goal_distance_m) &&
        std::isfinite(out.goal_yaw_error_rad),
      "path overflow must withhold authorization with finite fault diagnostics");
  }
  // A squared segment length can overflow while its length, projection and
  // local reference are all representable. Preserve the ordinary local result.
  for (double sign : {-1., 1.}) {
    ControllerInput input;
    input.vehicle.pose = {sign * .5, .1, 0};
    input.reference_path = {{0, 0, 0}, {sign * 1e160, 0, 0}};
    const auto path = PathManager(c).update(input);
    check(
      std::abs(path.progress_m - .5) < 1e-9 && std::abs(path.cross_track_error_m - .1) < 1e-9 &&
        std::abs(path.target.x - sign * 1.5) < 1e-9 && std::isfinite(path.remaining_m),
      "representable projection must not depend on squaring the segment length");
  }
  Config long_lookahead = c;
  long_lookahead.path_lookahead_m = 4e160;
  ControllerInput corner;
  corner.reference_path = {{0, 0, 0}, {1e160, 1e160, 0}, {0, 2e160, 0}};
  const auto reference = PathManager(long_lookahead).update(corner);
  check(
    reference.corner_target && !reference.goal_eligible && reference.target.x == 1e160 &&
      reference.target.y == 1e160,
    "overflowing unscaled dot/cross products cannot hide a blocking corner");

  ControllerInput perpendicular;
  perpendicular.vehicle.pose = {1e308, 0, 0};
  perpendicular.reference_path = {{0, 0, 0}, {0, 2e-12, 0}};
  const auto orthogonal = PathManager(c).update(perpendicular);
  check(
    orthogonal.progress_m == 0 && orthogonal.cross_track_error_m == 1e308,
    "a finite perpendicular projection must survive an overflowing offset-to-length ratio");

  long_lookahead.path_lookahead_m = 1e308;
  ControllerInput remote_target;
  remote_target.vehicle.pose = {1e308, 0, 0};
  remote_target.reference_path = {{0, 0, 0}, {-1e308, 0, 0}};
  bool rejected = false;
  try {
    PathManager(long_lookahead).update(remote_target);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "finite nearest error cannot authorize an overflowing local-target distance");

  PathManager manager(c);
  ControllerInput input;
  input.reference_path = {{-1e308, 0, 0}};
  input.vehicle.pose = input.reference_path.front();
  manager.update(input);
  input.vehicle.pose = {1e308, 0, 0};
  rejected = false;
  try {
    manager.update(input);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "overflowing displacement on a cached path must be rejected");
  input.vehicle.pose = input.reference_path.front();
  const auto recovered = manager.update(input);
  check(
    recovered.changed && recovered.progress_m == 0 && recovered.cross_track_error_m == 0,
    "derived-error rejection must clear partial matching state before the next valid observation");
}
int main()
{
  try {
    test_path_progress_and_replan();
    test_loops_duplicates_and_yaw();
    test_completion_requires_measured_stop();
    test_effective_target_and_goal_eligibility();
    test_task_restart_and_stall();
    test_cusp_target();
    test_short_paths_preserve_segment_order();
    test_dense_segment_capture();
    test_replan_execution_boundaries();
    test_path_numerical_boundaries();
    test_large_angles_and_derived_errors();
    std::cout << "Navigation regressions passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
