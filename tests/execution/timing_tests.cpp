#include <iostream>
#include <limits>

#include "behavior_fixture.hpp"
#include "execution/detail/joint_timing.hpp"
#include "planning/detail/planner.hpp"
#include "swerve_mppi/execution/timing.hpp"

using namespace swerve_mppi;
using namespace swerve_mppi::test;
namespace
{
detail::JointCommandEnvelope command(
  double now, std::uint64_t sequence = 1, std::uint64_t session = 10)
{
  JointCommand output;
  output.action = Action::Hold;
  ControllerInput source;
  source.reference_path = {{0, 0, 0}};
  return {session, sequence, now, output, now, now, now + .15, CommandTask::capture(source)};
}
CommandEnvelope timing_command(double now, std::uint64_t sequence = 1)
{
  return detail::timing_envelope(command(now, sequence));
}
VehicleState measured(double stamp)
{
  VehicleState state;
  state.stamp_s = stamp;
  state.time_in_mode_s = 2;
  return state;
}
TimedExecutionResult execute(
  detail::JointTimedExecutor & executor,
  const std::optional<detail::JointCommandEnvelope> & command, const VehicleState & state,
  double now)
{
  ControllerInput latest;
  latest.vehicle = state;
  latest.reference_path = {state.pose};
  return executor.update(command, latest, now);
}
void test_regular_and_jittered_ticks()
{
  Config c;
  detail::JointTimedExecutor executor(c, 10);
  for (int tick = 0; tick < 20; ++tick) {
    const double now = 1 + tick * c.dt_s + (tick % 2 == 0 ? 0 : .01);
    const auto result = execute(executor, command(now, tick + 1), measured(now), now);
    check(
      result.timing_error == TimingError::None && !result.execution.feedback.fault,
      "regular fresh ticks with bounded jitter must execute");
  }
}
void test_feedback_and_clock_failures()
{
  Config c;
  for (auto expected :
       {TimingError::ClockDiscontinuity, TimingError::OffPeriod, TimingError::FeedbackTimeout,
        TimingError::NonmonotonicFeedback, TimingError::InvalidTime}) {
    detail::JointTimedExecutor executor(c, 10);
    execute(executor, command(1), measured(1), 1);
    double now = 1.1, stamp = 1.1;
    if (expected == TimingError::ClockDiscontinuity) {
      now = stamp = .5;
    }
    if (expected == TimingError::OffPeriod) {
      now = stamp = 1.3;
    }
    if (expected == TimingError::FeedbackTimeout) {
      stamp = .9;
    }
    if (expected == TimingError::NonmonotonicFeedback) {
      stamp = 1;
    }
    if (expected == TimingError::InvalidTime) {
      stamp = 1.2;
    }
    const auto result = execute(executor, command(now, 2), measured(stamp), now);
    check(
      result.timing_error == expected && result.execution.feedback.fault &&
        result.execution.action == Action::SafeStop,
      "clock/freshness faults must stop execution with a distinct reason");
    const auto later = execute(executor, command(1.2, 3), measured(1.2), 1.2);
    check(
      later.timing_error == expected && later.execution.feedback.fault,
      "a later fresh sample must not silently clear a timing fault");
  }
  TimingGuard guard(c, 10);
  check(
    guard.check_feedback(std::numeric_limits<double>::quiet_NaN(), 1) == TimingError::InvalidTime,
    "nonfinite measurement time must be rejected");
}
void test_command_expiry_replay_and_loss()
{
  Config c;
  for (auto expected :
       {TimingError::CommandTimeout, TimingError::CommandReplay, TimingError::SessionMismatch,
        TimingError::MissingCommand, TimingError::InvalidTime}) {
    // Begin with positive Drive so loss/replay tests cannot merely repeat Hold.
    auto drive = command(1);
    drive.command.action = Action::Drive;
    drive.command.body_command.vx = .09;
    drive.command.wheel_speed_targets.fill(.09);
    detail::JointTimedExecutor driving(c, 10);
    check(
      execute(driving, drive, measured(1), 1).execution.action == Action::Drive,
      "watchdog test must begin with an accepted positive drive");
    drive.sequence = 2;
    drive.issued_at_s = 1.1;
    std::optional<detail::JointCommandEnvelope> next = drive;
    if (expected == TimingError::CommandTimeout) {
      next->issued_at_s = .9;
    }
    if (expected == TimingError::CommandReplay) {
      next->sequence = 1;
    }
    if (expected == TimingError::SessionMismatch) {
      next->session_id = 9;
    }
    if (expected == TimingError::MissingCommand) {
      next.reset();
    }
    if (expected == TimingError::InvalidTime) {
      next->issued_at_s = 1.2;
    }
    const auto result = execute(driving, next, measured(1.1), 1.1);
    check(
      result.timing_error == expected && result.execution.feedback.fault &&
        std::all_of(
          result.execution.wheel_speed_targets.begin(), result.execution.wheel_speed_targets.end(),
          [](double v) { return v == 0; }),
      "stale/replayed/missing commands must never authorize drive");
  }
  TimingGuard guard(c, 10);
  guard.check_feedback(1, 1);
  guard.check_command(timing_command(1), 1);
  guard.check_feedback(1.1, 1.1);
  check(
    guard.check_command(timing_command(.99, 2), 1.1) == TimingError::CommandReplay,
    "a newer sequence cannot authorize an older issued timestamp");
}
void test_clock_reset_requires_new_session_and_verified_stop()
{
  Config c;
  detail::JointTimedExecutor executor(c, 10);
  execute(executor, command(1), measured(1), 1);
  execute(executor, command(.1, 2), measured(.1), .1);
  auto recovered = measured(.1);
  bool rejected = false;
  try {
    executor.reset(recovered, 10);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "clock recovery must renew the transport session");
  recovered.wheel_speeds.fill(.1);
  rejected = false;
  try {
    executor.reset(recovered, 11);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "session renewal cannot bypass measured stopping");
  recovered.wheel_speeds.fill(0);
  recovered.time_in_mode_s = 0;
  executor.reset(recovered, 11);
  const auto delayed = execute(executor, command(.1, 3, 10), recovered, .1);
  check(
    delayed.timing_error == TimingError::SessionMismatch && delayed.execution.feedback.fault,
    "a delayed command from the old clock/session must be rejected after "
    "reset");
  executor.reset(recovered, 12);
  const auto fresh = execute(executor, command(.1, 1, 12), recovered, .1);
  check(
    fresh.timing_error == TimingError::None && !fresh.execution.feedback.fault,
    "verified recovery permits a fresh command sequence in a new session");
}
void test_mode_ids_survive_timing_recovery()
{
  Config c;
  detail::JointTimedExecutor executor(c, 10);
  auto envelope = command(1);
  envelope.command.action = Action::RequestMode;
  envelope.command.requested_mode = DriveMode::Crab;
  envelope.command.mode_request = JointModeRequest{
    7, DriveMode::Crab, DriveModel(c).steering_for_entry(DriveMode::Crab, {0, .2, 0}, {})};
  check(
    !execute(executor, envelope, measured(1), 1).execution.feedback.fault,
    "valid mode request must begin under the timing guard");
  auto recovered = measured(.1);
  recovered.time_in_mode_s = 0;
  executor.reset(recovered, 11);
  envelope.session_id = 11;
  envelope.issued_at_s = .1;
  envelope.source_stamp_s = .1;
  envelope.execute_at_s = .1;
  envelope.valid_until_s = .125;
  check(
    execute(executor, envelope, recovered, .1).execution.feedback.fault,
    "a new transport session must not permit an old mode request ID");
}
void test_timing_limits_and_boundaries()
{
  Config c;
  for (auto bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    TimingLimits limits;
    limits.max_feedback_age_s = bad;
    bool rejected = false;
    try {
      TimingGuard guard(c, 10, limits);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected, "nonpositive/nonfinite timing limits must be rejected");
  }
  TimingGuard guard(c, 10);
  check(
    guard.check_feedback(.85, 1) == TimingError::None &&
      guard.check_command(timing_command(.85), 1) == TimingError::None,
    "feedback/command age exactly at the configured bound must be accepted");
  guard.check_feedback(.95, 1.1);
  check(
    guard.check_command(timing_command(.95, 2), 1.1) == TimingError::None,
    "fixed-age delayed feedback must remain valid at regular periods");
  check(
    guard.check_command(timing_command(1, 3), 1.1) == TimingError::CommandReplay,
    "two commands cannot be accepted for the same model tick");
}
void test_guarded_closed_loop_and_idempotent_requests()
{
  Config c;
  c.samples_per_branch = 8;
  detail::Planner controller(c);
  detail::JointTimedExecutor executor(c, 10);
  ControllerInput in;
  in.vehicle = measured(1);
  in.reference_path = {{0, 0, 0}, {0, .15, 0}};
  std::optional<JointModeRequest> request;
  bool complete = false;
  for (int tick = 0; tick < 100; ++tick) {
    const auto output = controller.compute(in);
    if (output.mode_request) {
      if (!request) {
        request = output.mode_request;
      }
      check(
        request->id == output.mode_request->id &&
          request->steering_targets == output.mode_request->steering_targets,
        "mode retries retain payload while transport sequence advances");
    }
    detail::JointCommandEnvelope envelope{
      10,
      static_cast<std::uint64_t>(tick + 1),
      in.vehicle.stamp_s,
      output,
      in.vehicle.stamp_s,
      in.vehicle.stamp_s,
      in.vehicle.stamp_s + .025,
      CommandTask::capture(in)};
    const auto result = executor.update(envelope, in, in.vehicle.stamp_s);
    check(
      result.timing_error == TimingError::None &&
        result.safety_error == ExecutionSafetyError::None && result.actuation &&
        !result.execution.feedback.fault,
      "guarded controller/executor loop must preserve mode handshake and "
      "capture");
    if (output.goal_reached) {
      complete = true;
      break;
    }
    actuate(in.vehicle, result.execution, c);
  }
  check(complete && request.has_value(), "guarded lateral task must switch, drive and settle");
}

void test_fresh_unconfirmed_feedback()
{
  Config c;
  detail::JointTimedExecutor executor(c, 10);
  execute(executor, command(1), measured(1), 1);
  auto state = measured(1.1);
  state.mode_confirmed = false;
  auto drive = command(1.1, 2);
  drive.command.action = Action::Drive;
  drive.command.body_command.vx = .05;
  drive.command.wheel_speed_targets.fill(.05);
  const auto rejected = execute(executor, drive, state, 1.1);
  check(
    rejected.timing_error == TimingError::None && rejected.execution.feedback.fault &&
      rejected.execution.action == Action::SafeStop,
    "fresh timing cannot authorize Drive when Stable mode confirmation "
    "disappears");
}

}  // namespace
int main()
{
  try {
    test_fresh_unconfirmed_feedback();
    test_regular_and_jittered_ticks();
    test_feedback_and_clock_failures();
    test_command_expiry_replay_and_loss();
    test_clock_reset_requires_new_session_and_verified_stop();
    test_mode_ids_survive_timing_recovery();
    test_timing_limits_and_boundaries();
    test_guarded_closed_loop_and_idempotent_requests();
    std::cout << "Timing regressions passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
