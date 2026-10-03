#include <chrono>
#include <iostream>
#include <limits>

#include "behavior_fixture.hpp"
#include "swerve_mppi/execution/profile_runner.hpp"
#include "swerve_mppi/feedback/feedback_adapter.hpp"
#include "swerve_mppi/planning/controller.hpp"
using namespace swerve_mppi;
using swerve_mppi::test::check;
JointObservation observation(const VehicleState & s, const Config & c)
{
  JointObservation j;
  j.stamp_s = s.stamp_s;
  // Deliberately interleaved/reversed, not canonical JointState order.
  for (const std::string corner : {"rr", "rl", "fr", "fl"}) {
    j.names.push_back("bot_" + corner + "_wheel_joint");
    j.names.push_back("bot_" + corner + "_steering_joint");
  }
  for (int i = 3; i >= 0; --i) {
    j.positions.push_back(0);
    j.positions.push_back(s.steering_angles[i]);
    j.velocities.push_back(s.wheel_speeds[i] / c.wheel_radius_m);
    j.velocities.push_back(0);
  }
  return j;
}
int main()
{
  try {
    Config c;
    c.compute_budget_ratio = 0;
    FeedbackAdapter adapter(c, "bot_");
    VehicleState s;
    s.stamp_s = 1;
    s.wheel_speeds.fill(.2);
    auto j = observation(s, c);
    const StampedPose pose{{0, 0, 1}, 1};
    auto result = adapter.make(j, pose, {}, 1);
    check(
      result.state && std::abs(result.state->velocity.vx - .2) < 1e-12 &&
        result.state->velocity.vy == 0 && result.state->wheel_speeds[0] == .2,
      "named encoder conversion must produce body twist, not rotated world "
      "twist");
    check(
      adapter.make(j, pose, {}, 1.001).error == SnapshotError::Unsynchronized,
      "raw snapshots cannot be restamped to application time");
    check(
      adapter.make(j, {pose.pose, .99}, {}, 1).error == SnapshotError::Unsynchronized,
      "pose and encoder stamp must agree");
    {
      auto exact = j;
      exact.stamp_s = 1 + 5000000 * 1e-9;
      const double clock = 1005000000LL * 1e-9;
      check(exact.stamp_s != clock, "regression must exercise different floating conversions");
      check(
        adapter.make(exact, {pose.pose, clock}, {}, clock).state.has_value(),
        "same source time with conversion roundoff must be admitted");
      check(
        !adapter.make(exact, {pose.pose, clock + 1e-9}, {}, clock).state,
        "floating conversion allowance cannot hide a real nanosecond "
        "mismatch");
      exact.stamp_ns = 1005000000LL;
      StampedPose stamped{pose.pose, clock, exact.stamp_ns};
      const auto ns = adapter.make_at_nanoseconds(exact, stamped, {}, exact.stamp_ns);
      check(
        ns.state &&
          ns.state->stamp_s ==
            std::chrono::duration<double>(std::chrono::nanoseconds(exact.stamp_ns)).count(),
        "integer ingress compares stamps before one canonical conversion");
      check(
        !adapter.make_at_nanoseconds(exact, stamped, {}, exact.stamp_ns + 1).state,
        "integer ingress rejects an adjacent application nanosecond");
      ++stamped.stamp_ns;
      check(
        !adapter.make_at_nanoseconds(exact, stamped, {}, exact.stamp_ns).state,
        "integer ingress rejects an adjacent pose nanosecond");
      // Adjacent nanoseconds collapse to the same double near a Unix epoch.
      exact.stamp_ns = 2000000000000000000LL;
      stamped.stamp_ns = exact.stamp_ns + 1;
      check(
        !adapter.make_at_nanoseconds(exact, stamped, {}, exact.stamp_ns).state,
        "integer comparison must precede conversion at large epochs");
      stamped.stamp_ns = exact.stamp_ns;
      exact.stamp_s = stamped.stamp_s = std::numeric_limits<double>::quiet_NaN();
      check(
        adapter.make_at_nanoseconds(exact, stamped, {}, exact.stamp_ns).state.has_value(),
        "integer ingress does not depend on independently converted seconds");
      exact.stamp_ns = -1;
      check(
        adapter.make_at_nanoseconds(exact, stamped, {}, 0).error == SnapshotError::InvalidTime,
        "integer ingress requires original nonnegative source stamps");
    }
    j.names[0] = j.names[2];
    check(!adapter.make(j, pose, {}, 1).state, "duplicate/missing required joint rejected");
    j = observation(s, c);
    j.velocities.pop_back();
    check(!adapter.make(j, pose, {}, 1).state, "partial joint vector rejected");
    j = observation(s, c);
    j.velocities[0] = std::numeric_limits<double>::quiet_NaN();
    check(!adapter.make(j, pose, {}, 1).state, "invalid encoder rejected");
    j = observation(s, c);
    j.positions[1] = 2;
    check(
      adapter.make(j, pose, {}, 1).error == SnapshotError::InvalidFeedback,
      "out-of-range measured steering rejected");
    j = observation(s, c);
    j.names.resize(65);
    j.positions.resize(65);
    j.velocities.resize(65);
    check(!adapter.make(j, pose, {}, 1).state, "adapter workload is bounded");
    j = observation(s, c);
    ModeFeedback pending{DriveMode::Crab, false, false, 42, .25};
    pending.accepted_mode_request = JointModeRequest{42, DriveMode::Crab, {}, {.2, 0, 0}};
    result = adapter.make(j, pose, pending, 1);
    check(
      result.state && !result.state->mode_confirmed && result.state->mode_request_id == 42 &&
        result.state->actual_mode == DriveMode::Crab && result.state->accepted_mode_request &&
        result.state->accepted_mode_request->entry_velocity.vx == .2,
      "adapter must preserve executor mode metadata without inventing "
      "confirmation");
    pending.accepted_mode_request->id = 41;
    check(
      adapter.make(j, pose, pending, 1).error == SnapshotError::InvalidFeedback,
      "geometry receipt must be bound to the echoed request ID");
    // Real guarded plan -> high-rate runner -> missing next plan. Recovery must
    // reset both protocols, and an old-session command must remain rejected.
    Controller controller(c);
    TimedExecutor executor(c, 1);
    ProfileRunner runner(c, .2);
    auto input = swerve_mppi::test::scenario_input("straight");
    const double now = input.vehicle.stamp_s;
    CommandEnvelope command{1,   1,   now,        controller.compute(input),
                            now, now, now + .025, CommandTask::capture(input)};
    auto guarded = executor.update(command, input, now);
    check(runner.install(guarded, now, 10), "install checked profile");
    check(runner.sample(now + .05, 10.05).has_value(), "sample real drive profile");
    check(
      !runner.sample(now + c.dt_s + .001, 10.101) && runner.fault(),
      "planner loss must not retain endpoint drive");
    runner.reset(input.vehicle);
    executor.reset(input.vehicle, 2);
    check(
      executor.update(command, input, now).timing_error == TimingError::SessionMismatch,
      "recovery cannot accept old-session command");
    ProfileRunner paused(c, .2);
    check(paused.install(guarded, now, 20), "install before pause");
    check(!paused.sample(now, 20.201), "paused sim cannot freeze wall watchdog");
    std::cout << "snapshot admission, profile loss, pause and recovery passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
