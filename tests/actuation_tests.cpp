#include "behavior_fixture.hpp"
#include "swerve_mppi/actuation.hpp"
#include "swerve_mppi/timing.hpp"
#include <iostream>
#include <limits>

using namespace swerve_mppi;
using namespace swerve_mppi::test;
namespace {
bool close(double a, double b, double tolerance = 1e-9) {
  return std::abs(a - b) <= tolerance;
}
ControllerInput input(double stamp = 1) {
  ControllerInput in;
  in.vehicle.stamp_s = stamp;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 0}, {5, 0, 0}};
  return in;
}
Output drive(const VehicleState &state, const Config &c, const Control &intent) {
  const auto step = DriveModel(c).step(state, intent, c.dt_s);
  check(step.valid && !step.aligning, "test Drive must have a feasible joint interval");
  Output out;
  out.action = Action::Drive;
  out.requested_mode = state.actual_mode;
  out.body_command = step.state.velocity;
  out.steering_targets = step.steering_targets;
  out.wheel_speed_targets = step.wheel_speed_targets;
  return out;
}
CommandEnvelope envelope(const ControllerInput &origin, const Output &out, double source,
                         double issued, double application,
                         std::uint64_t sequence = 1) {
  return {10, sequence, issued, out, source, application, application + .025,
          CommandTask::capture(origin)};
}
// Independent encoder field, not Kinematics::forward or DriveModel integration.
Twist2d encoder(const ActuatorTargets &joints, const Config &c) {
  const double x[] = {c.wheelbase_m / 2, c.wheelbase_m / 2, -c.wheelbase_m / 2, -c.wheelbase_m / 2};
  const double y[] = {c.track_m / 2, -c.track_m / 2, c.track_m / 2, -c.track_m / 2};
  Twist2d out;
  double denominator = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    const double vx = joints.wheel_speeds[i] * std::cos(joints.steering_angles[i]);
    const double vy = joints.wheel_speeds[i] * std::sin(joints.steering_angles[i]);
    out.vx += vx / 4;
    out.vy += vy / 4;
    out.wz += x[i] * vy - y[i] * vx;
    denominator += x[i] * x[i] + y[i] * y[i];
  }
  out.wz /= denominator;
  return out;
}
void test_drive_profile_and_independent_motion() {
  Config c;
  for (auto mode : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
    auto in = input();
    in.vehicle.actual_mode = mode;
    const Control before = mode == DriveMode::Spin ? Control{0, 0, .2}
                           : mode == DriveMode::Crab ? Control{.2, .1, 0} : Control{.4, 0, .2};
    const auto joints = Kinematics(c).inverse(before, {});
    in.vehicle.steering_angles = joints.angles;
    in.vehicle.wheel_speeds = joints.speeds;
    in.vehicle.velocity = {before.vx, before.vy, before.wz};
    const Control intent = mode == DriveMode::Spin ? Control{0, 0, .3}
                           : mode == DriveMode::Crab ? Control{.3, .14, 0} : Control{.45, 0, .3};
    const auto command = drive(in.vehicle, c, intent);
    ModeExecutor executor(c, mode);
    const auto execution = executor.update(command, in.vehicle);
    const auto plan = ActuationModel(c).plan(in.vehicle, execution);
    check(plan.has_value(), "healthy supervisor Drive must produce a checked actuator plan");
    for (double fraction : {0.0, .1, .25, .5, .9, 1.0}) {
      const auto targets = plan->sample(fraction * c.dt_s);
      check(targets.has_value(), "in-interval Drive samples must exist");
      for (std::size_t i = 0; i < 4; ++i) {
        check(close(targets->wheel_speeds[i], joints.speeds[i] + fraction *
                        (execution.wheel_speed_targets[i] - joints.speeds[i])) &&
                  close(targets->steering_angles[i], joints.angles[i] + fraction *
                        (execution.steering_targets[i] - joints.angles[i])),
              "Drive must reach each target by affine full-tick interpolation");
      }
    }
    Pose2d oracle = in.vehicle.pose;
    constexpr int subdivisions = 4096;
    const double h = c.dt_s / subdivisions;
    for (int j = 0; j < subdivisions; ++j) {
      const auto targets = plan->sample((j + .5) * h);
      const auto velocity = encoder(*targets, c);
      const double yaw = oracle.yaw + velocity.wz * h / 2;
      oracle.x += (std::cos(yaw) * velocity.vx - std::sin(yaw) * velocity.vy) * h;
      oracle.y += (std::sin(yaw) * velocity.vx + std::cos(yaw) * velocity.vy) * h;
      oracle.yaw = wrap_angle(oracle.yaw + velocity.wz * h);
      const auto &from = plan->start().pose;
      const auto &to = plan->endpoint().state.pose;
      const double dx = to.x - from.x, dy = to.y - from.y;
      const double length2 = dx * dx + dy * dy;
      const double fraction = length2 > 0 ? std::clamp(
          ((oracle.x - from.x) * dx + (oracle.y - from.y) * dy) / length2, 0.0, 1.0) : 0;
      check(std::hypot(oracle.x - from.x - fraction * dx,
                       oracle.y - from.y - fraction * dy) <= plan->endpoint().sweep_margin_m + 1e-8,
            "independent intermediate motion must stay inside the certified swept enclosure");
      check(std::hypot(velocity.vx, velocity.vy) <=
                std::max(c.max_vx_mps, c.max_crab_speed_mps) + 1e-9 &&
                std::abs(velocity.wz) <= std::max(c.max_yaw_rate_radps, c.max_spin_radps) + 1e-9,
            "sampled physical encoder field must stay inside speed caps");
    }
    const auto &end = plan->endpoint();
    check(std::hypot(end.state.pose.x - oracle.x, end.state.pose.y - oracle.y) <=
              end.integration_error_m + 1e-8 &&
              std::abs(angle_distance(end.state.pose.yaw, oracle.yaw)) < 1e-8,
          "checked interval must enclose the independently integrated sample stream");
    check(!plan->sample(-.001) && !plan->sample(c.dt_s + .001) &&
              !plan->sample(std::numeric_limits<double>::quiet_NaN()),
          "expired or invalid actuator samples must not extrapolate");
    auto other = c;
    other.max_linear_decel_mps2 *= .5;
    Trajectory incompatible;
    RolloutEngine(other).generate_execution(*plan, incompatible);
    check(!incompatible.valid,
          "a checked interval cannot be reused under different actuator model assumptions");
  }
}
void test_proportional_brake_and_phased_alignment() {
  Config c;
  ActuationModel model(c);
  auto in = input();
  in.vehicle.wheel_speeds = {.3, .1, -.2, .4};
  in.vehicle.steering_angles = {.2, -.2, .3, -.3};
  in.vehicle.velocity = encoder({in.vehicle.steering_angles, in.vehicle.wheel_speeds}, c);
  Output brake;
  brake.action = Action::Brake;
  ModeExecutor executor(c);
  const auto plan = model.plan(in.vehicle, executor.update(brake, in.vehicle));
  check(plan.has_value() && plan->braking_duration_s() > 0,
        "nonideal measured wheel pairs must admit a proportional brake profile");
  for (double t : {0.0, .025, .05, .075, .1}) {
    const auto targets = plan->sample(t);
    for (std::size_t i = 0; i < 4; ++i)
      check(close(targets->wheel_speeds[i] / in.vehicle.wheel_speeds[i],
                  std::max(0.0, 1 - t / plan->braking_duration_s())) &&
                targets->steering_angles[i] == in.vehicle.steering_angles[i],
            "Brake keeps measured steering and preserves wheel speed proportions");
  }
  // A handover threshold does not mean the wheels have reached physical zero.
  in = input();
  in.vehicle.wheel_speeds.fill(.004);
  in.vehicle.velocity.vx = .004;
  Output hold;
  hold.action = Action::Hold;
  hold.steering_targets.fill(.4);
  ModeExecutor holder(c);
  const auto alignment = model.plan(in.vehicle, holder.update(hold, in.vehicle));
  check(alignment && close(alignment->braking_duration_s(), .004),
        "residual rolling below stopped thresholds still consumes braking time");
  const auto braking = alignment->sample(.002);
  const auto stopped = alignment->sample(.004);
  const auto steering = alignment->sample(.005);
  const auto end = alignment->sample(c.dt_s);
  check(braking->steering_angles[0] == 0 && close(braking->wheel_speeds[0], .002) &&
            stopped->steering_angles[0] == 0 && stopped->wheel_speeds[0] == 0 &&
            close(steering->steering_angles[0], .0025) && steering->wheel_speeds[0] == 0 &&
            close(end->steering_angles[0], c.max_steer_rate_radps * (.1 - .004)),
        "Hold must finish complete braking before stationary steering alignment");
  RolloutEngine rollout(c);
  Trajectory trace;
  rollout.generate_execution(*alignment, trace);
  check(trace.valid && close(trace.final_state.pose.x, .004 * .004 / 2) &&
            trace.final_state.wheel_speeds[0] == 0,
        "stopping continuation must include the residual displacement exactly");
  ExecutionResult disabled;
  check(!model.plan(in.vehicle, disabled),
        "SafeStop must not claim a guaranteed normal-braking trajectory");
  auto malformed = holder.update(hold, input(1.1).vehicle);
  malformed.wheel_speed_targets[0] = std::numeric_limits<double>::quiet_NaN();
  check(!model.plan(input(1.1).vehicle, malformed), "nonfinite stopping targets must be rejected");
}
void test_planning_stop_preserves_feedback_and_full_braking() {
  Config c;
  ActuationModel model(c);
  RolloutEngine rollout(c);
  auto in = input();
  in.vehicle.mode_confirmed = false;
  in.vehicle.mode_request_id = 17;
  const std::array<double, 4> targets{.4, .4, .4, .4};
  for (double speed : {0.0, .004, .3}) {
    in.vehicle.wheel_speeds.fill(speed);
    in.vehicle.velocity = {speed, 0, 0};
    for (auto action : {Action::Brake, Action::Hold, Action::RequestMode}) {
      const auto plan = model.plan_stopping(in.vehicle, action, targets);
      check(plan.has_value(), "unconfirmed feedback must permit nominal non-driving checks");
      Trajectory trace;
      rollout.generate_execution(*plan, trace);
      const double expected_angle = action != Action::Brake && speed <= c.stopped_wheel_speed_mps
          ? std::min(.4, c.max_steer_rate_radps * (c.dt_s - speed / c.max_linear_decel_mps2))
          : 0;
      const auto end = plan->sample(c.dt_s);
      check(trace.valid && close(trace.final_state.pose.x, speed * speed /
                                  (2 * c.max_linear_decel_mps2)) &&
                trace.final_state.wheel_speeds == std::array<double, 4>{} &&
                close(end->steering_angles[0], expected_angle) &&
                close(trace.final_state.steering_angles[0], expected_angle) &&
                !trace.final_state.mode_confirmed && !trace.final_state.mode_fault &&
                trace.final_state.mode_request_id == 17 &&
                trace.final_state.actual_mode == in.vehicle.actual_mode,
            "planning checks must retain feedback, full braking distance and phased steering");
    }
  }
  check(!model.plan_stopping(in.vehicle, Action::Drive, targets) &&
            !model.plan_stopping(in.vehicle, Action::SafeStop, targets),
        "a nominal stopping check cannot authorize Drive or certify emergency dynamics");
  auto bad_targets = targets;
  bad_targets[0] = std::numeric_limits<double>::quiet_NaN();
  check(!model.plan_stopping(in.vehicle, Action::Hold, bad_targets),
        "nonfinite steering cannot produce a nominal plan");
  in.vehicle.mode_fault = true;
  check(!model.plan_stopping(in.vehicle, Action::Hold, targets),
        "faulted measured feedback cannot produce a normal stopping plan");
  in.vehicle.mode_fault = false;
  in.vehicle.stamp_s = -1;
  check(!model.plan_stopping(in.vehicle, Action::Brake, targets),
        "invalid measured feedback cannot produce a normal stopping plan");
}
void test_delayed_drive_revalidated_at_latest_pose() {
  Config c;
  auto old = input();
  old.vehicle.velocity.vx = .8;
  old.vehicle.wheel_speeds.fill(.8);
  old.obstacles = {{1.01, 0, .05}};
  const auto command = drive(old.vehicle, c, {.8, 0, 0});
  TimedExecutor executor(c, 10);
  const auto first = executor.update(envelope(old, command, 1, 1, 1), old, 1);
  check(first.actuation && first.execution.action == Action::Drive,
        "old position must authorize the independently safe first Drive");
  Trajectory trace;
  RolloutEngine(c).generate_execution(*first.actuation, trace);
  check(trace.valid && close(trace.final_state.pose.x, .4),
        "old one-Drive-plus-stop endpoint must be 0.4 m");
  auto latest = old;
  latest.vehicle.pose.x = .08;
  latest.vehicle.stamp_s = 1.1;
  const auto rejected = executor.update(envelope(old, command, 1, 1.03, 1.1, 2), latest, 1.1);
  check(rejected.timing_error == TimingError::None &&
            rejected.safety_error == ExecutionSafetyError::CommandRejected &&
            rejected.rejected_status == TrajectoryStatus::Collision &&
            rejected.execution.action == Action::Brake && rejected.actuation &&
            !rejected.execution.feedback.fault,
        "a recent delayed Drive must be replaced by a freshly validated complete stop");
  RolloutEngine(c).generate_execution(*rejected.actuation, trace);
  check(trace.valid && close(trace.final_state.pose.x, .4),
        "latest brake-only endpoint remains safe at 0.4 m");
  latest.vehicle = rejected.actuation->endpoint().state;
  latest.obstacles.clear();
  const auto next = drive(latest.vehicle, c, {.7, 0, 0});
  const double now = latest.vehicle.stamp_s;
  const auto recovered = executor.update(envelope(latest, next, now, now, now, 3), latest, now);
  check(recovered.execution.action == Action::Drive && recovered.actuation &&
            recovered.safety_error == ExecutionSafetyError::None,
        "a blocked command with safe braking must allow recovery on fresh context");
}
void test_latest_obstacles_and_unsafe_stop_latch() {
  Config c;
  auto in = input();
  const auto command = drive(in.vehicle, c, {.2, 0, 0});
  const auto queued = envelope(in, command, 1, 1, 1);
  in.obstacles = {{.606, 0, .05}};
  TimedExecutor executor(c, 10);
  const auto blocked = executor.update(queued, in, 1);
  check(blocked.safety_error == ExecutionSafetyError::CommandRejected && blocked.actuation &&
            blocked.execution.action == Action::Brake,
        "new obstacles must reject a previously clear Drive while preserving a safe stop");
  in = input();
  in.vehicle.velocity.vx = .8;
  in.vehicle.wheel_speeds.fill(.8);
  in.vehicle.pose.x = .12;
  in.obstacles = {{1.01, 0, .05}};
  TimedExecutor unsafe(c, 10);
  const auto fault = unsafe.update(envelope(in, drive(in.vehicle, c, {.8, 0, 0}), 1, 1, 1), in, 1);
  check(fault.safety_error == ExecutionSafetyError::UnsafeStoppingTrajectory &&
            fault.execution.action == Action::SafeStop && fault.execution.feedback.fault &&
            !fault.actuation,
        "an unsafe current stop must latch SafeStop without a certified actuator profile");
  in.obstacles.clear();
  in.vehicle.stamp_s = 1.1;
  const auto later = unsafe.update(envelope(in, command, 1.1, 1.1, 1.1, 2), in, 1.1);
  check(later.execution.feedback.fault && !later.actuation,
        "fresh obstacles or commands must not clear an unsafe-stop fault");
  c.stopping_horizon_steps = 2;
  TimedExecutor bounded(c, 10);
  const auto exhausted = bounded.update(
      envelope(in, drive(in.vehicle, c, {.8, 0, 0}), 1.1, 1.1, 1.1), in, 1.1);
  check(exhausted.safety_error == ExecutionSafetyError::UnsafeStoppingTrajectory,
        "execution stopping-budget exhaustion must fail closed");
}
class GateSteering final : public TrajectoryConstraint {
public:
  bool allows(const ControllerInput &in, const Trajectory &trajectory) const override {
    return in.path_id == 9 || trajectory.final_state.steering_angles == in.vehicle.steering_angles;
  }
};
void test_commands_bound_to_originating_task() {
  Config c;
  for (int change = 0; change < 10; ++change) {
    auto origin = input();
    origin.reference_path.insert(origin.reference_path.begin() + 1, {2, 0, 0});
    origin.vehicle.velocity.vx = .2;
    origin.vehicle.wheel_speeds.fill(.2);
    const auto output = drive(origin.vehicle, c, {.2, 0, 0});
    auto queued = envelope(origin, output, 1, 1.03, 1.1);
    // Mutate the original input after capture: the queued snapshot must own its
    // geometry, not alias a path that an adapter can overwrite during replanning.
    origin.vehicle.stamp_s = 1.1;
    origin.vehicle.pose.x = .02;
    if (change == 0)
      ++origin.path_id;
    else if (change == 1)
      origin.reference_path.back().x = -5;
    else if (change == 2)
      origin.reference_path[1].x += .1;
    else if (change == 3)
      origin.reference_path[1].y += .1;
    else if (change == 4)
      origin.reference_path.back().yaw += .1;
    else if (change == 5)
      origin.reference_path[1].yaw += .1;
    else if (change == 6)
      origin.heading_policy = PathHeadingPolicy::GoalOnly;
    else if (change == 7)
      origin.reference_path.erase(origin.reference_path.begin() + 1);
    else if (change == 8)
      std::swap(origin.reference_path[0], origin.reference_path[1]);
    else
      queued.source_task.reset();

    // Every obsolete command remains mechanically and collision valid. The
    // rejection must come from task identity, not an incidental safety failure.
    ModeExecutor preview(c);
    const auto plan = ActuationModel(c).plan(origin.vehicle, preview.update(output, origin.vehicle));
    check(plan.has_value(), "stale task reproduction must have valid Drive targets");
    Trajectory trace;
    RolloutEngine(c).generate_execution(*plan, trace);
    check(TrajectoryValidator(c).check(origin, trace) == TrajectoryStatus::Valid,
          "collision checks alone must permit the stale task reproduction");
    const double obsolete_stop_x = trace.final_state.pose.x;
    TimedExecutor executor(c, 10);
    const auto rejected = executor.update(queued, origin, 1.1);
    check(rejected.timing_error == TimingError::None &&
              rejected.safety_error == ExecutionSafetyError::TaskMismatch &&
              rejected.rejected_status == TrajectoryStatus::Invalid &&
              rejected.execution.action == Action::Brake && rejected.actuation &&
              !rejected.execution.feedback.fault,
          "changed ID, full geometry, heading policy or missing snapshot must reject Drive");
    RolloutEngine(c).generate_execution(*rejected.actuation, trace);
    check(trace.valid && trace.final_state.wheel_speeds[0] == 0 &&
              TrajectoryValidator(c).check(origin, trace) == TrajectoryStatus::Valid &&
              trace.final_state.pose.x < obsolete_stop_x,
          "a replaced task may execute only its separately checked stopping fallback");
    origin.vehicle = rejected.actuation->endpoint().state;
    Output hold;
    hold.action = Action::Hold;
    const double now = origin.vehicle.stamp_s;
    const auto fresh = executor.update(envelope(origin, hold, now, now, now, 2), origin, now);
    check(fresh.safety_error == ExecutionSafetyError::None && fresh.actuation &&
              !fresh.execution.feedback.fault,
          "a fresh command bound to the current task must recover without reset");
  }
}
void test_stale_task_request_does_not_commit() {
  Config c;
  auto old = input();
  Output request;
  request.action = Action::RequestMode;
  request.requested_mode = DriveMode::Crab;
  request.mode_request = ModeRequest{7, DriveMode::Crab,
      DriveModel(c).steering_for_entry(DriveMode::Crab, {0, .2, 0}, {})};
  const auto queued = envelope(old, request, 1, 1.03, 1.1);
  auto latest = old;
  latest.vehicle.stamp_s = 1.1;
  ++latest.path_id;
  TimedExecutor executor(c, 10);
  const auto rejected = executor.update(queued, latest, 1.1);
  check(rejected.safety_error == ExecutionSafetyError::TaskMismatch && rejected.actuation &&
            rejected.execution.feedback.request_id == 0 &&
            rejected.execution.phase == TransitionPhase::Stable,
        "task rejection must precede mode-request preview or state commitment");
  latest.vehicle = rejected.actuation->endpoint().state;
  const double now = latest.vehicle.stamp_s;
  const auto fresh = executor.update(envelope(latest, request, now, now, now, 2), latest, now);
  check(fresh.safety_error == ExecutionSafetyError::None && fresh.actuation &&
            fresh.execution.feedback.request_id == 7,
        "the same request ID must remain usable by a new command for the current task");

  old.vehicle.velocity.vx = .8;
  old.vehicle.wheel_speeds.fill(.8);
  auto unsafe = old;
  ++unsafe.path_id;
  unsafe.vehicle.stamp_s = 1.15;
  unsafe.vehicle.pose.x = .12;
  unsafe.obstacles = {{1.01, 0, .05}};
  TimedExecutor fail_closed(c, 10);
  const auto fault = fail_closed.update(
      envelope(old, drive(old.vehicle, c, {.8, 0, 0}), 1, 1.1, 1.15), unsafe, 1.15);
  check(fault.safety_error == ExecutionSafetyError::UnsafeStoppingTrajectory &&
            fault.execution.feedback.fault && !fault.actuation,
        "task mismatch must not bypass validation of an unsafe current stopping trajectory");
}
void test_request_rejection_is_transactional() {
  Config c;
  auto validator = std::make_shared<TrajectoryValidator>(c);
  validator->add(std::make_shared<GateSteering>());
  TimedExecutor executor(c, 10, DriveMode::DualAckermann, {}, validator);
  auto in = input();
  Output request;
  request.action = Action::RequestMode;
  request.requested_mode = DriveMode::Crab;
  request.mode_request = ModeRequest{7, DriveMode::Crab,
      DriveModel(c).steering_for_entry(DriveMode::Crab, {0, .2, 0}, {})};
  const auto blocked = executor.update(envelope(in, request, 1, 1, 1), in, 1);
  check(blocked.safety_error == ExecutionSafetyError::CommandRejected &&
            blocked.execution.feedback.request_id == 0 &&
            blocked.execution.phase == TransitionPhase::Stable,
        "rejecting a new request must not consume its ID or commit its transition");
  in.path_id = 9;
  in.vehicle.stamp_s = 1.1;
  const auto accepted = executor.update(envelope(in, request, 1.1, 1.1, 1.1, 2), in, 1.1);
  check(accepted.actuation && !accepted.execution.feedback.fault &&
            accepted.execution.feedback.request_id == 7,
        "the same mode request ID remains eligible after a transactional safety rejection");
  bool rejected = false;
  auto incompatible = c;
  incompatible.robot_radius_m += .1;
  try {
    TimedExecutor mismatch(c, 10, DriveMode::DualAckermann, {},
                           std::make_shared<TrajectoryValidator>(incompatible));
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "execution and planning validators must use compatible safety limits");
}
void test_scheduling_metadata_and_current_state() {
  Config c;
  Output hold;
  hold.action = Action::Hold;
  for (auto expected : {TimingError::InvalidPlanTime, TimingError::SourceTimeout,
                        TimingError::NotYetExecutable, TimingError::ExecutionExpired}) {
    auto queued = envelope(input(), hold, .99, .99, 1);
    if (expected == TimingError::InvalidPlanTime)
      queued.source_stamp_s = 1;
    if (expected == TimingError::SourceTimeout)
      queued.source_stamp_s = .7;
    if (expected == TimingError::NotYetExecutable) {
      queued.execute_at_s = 1.01;
      queued.valid_until_s = 1.02;
    }
    if (expected == TimingError::ExecutionExpired) {
      queued.execute_at_s = .99;
      queued.valid_until_s = .999;
    }
    TimedExecutor executor(c, 10);
    const auto result = executor.update(queued, input(), 1);
    check(result.timing_error == expected && result.execution.feedback.fault && !result.actuation,
          "invalid scheduling/source timestamps must fail closed with a distinct reason");
  }
  for (int field = 0; field < 3; ++field) {
    auto queued = envelope(input(), hold, 1, 1, 1);
    (field == 0 ? queued.source_stamp_s : field == 1 ? queued.execute_at_s : queued.valid_until_s) =
        std::numeric_limits<double>::quiet_NaN();
    TimedExecutor executor(c, 10);
    check(executor.update(queued, input(), 1).timing_error == TimingError::InvalidPlanTime,
          "every required plan timestamp must be finite");
  }
  TimedExecutor missing(c, 10);
  check(missing.update(CommandEnvelope{10, 1, 1, hold}, input(), 1).timing_error ==
            TimingError::InvalidPlanTime,
        "legacy envelopes without source/scheduling metadata must not authorize motion");
  TimedExecutor stale(c, 10);
  const auto raw = stale.update(envelope(input(.99), hold, .99, 1, 1), input(.99), 1);
  check(raw.timing_error == TimingError::None &&
            raw.safety_error == ExecutionSafetyError::StateNotCurrent &&
            raw.execution.feedback.fault,
        "a recent raw observation must not be silently treated as the execution-start pose");
  TimedExecutor invalid(c, 10);
  auto bad = input();
  bad.obstacles.push_back({0, 0, -1});
  check(invalid.update(envelope(bad, hold, 1, 1, 1), bad, 1).safety_error ==
            ExecutionSafetyError::InvalidContext,
        "invalid current obstacle data must fail closed");
}
} // namespace
int main() {
  try {
    test_drive_profile_and_independent_motion();
    test_proportional_brake_and_phased_alignment();
    test_planning_stop_preserves_feedback_and_full_braking();
    test_delayed_drive_revalidated_at_latest_pose();
    test_latest_obstacles_and_unsafe_stop_latch();
    test_commands_bound_to_originating_task();
    test_stale_task_request_does_not_commit();
    test_request_rejection_is_transactional();
    test_scheduling_metadata_and_current_state();
    std::cout << "Actuation and execution-safety regressions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
