#include <iostream>
#include <limits>
#include <type_traits>
#include <utility>

#include "profile_fixture.hpp"
#include "swerve_mppi/execution/chassis_executor.hpp"
#include "swerve_mppi/execution/timing.hpp"
#include "swerve_mppi/planning/controller.hpp"
#include "swerve_mppi/planning/mode.hpp"

using namespace swerve_mppi;
using namespace swerve_mppi::test;
namespace
{
template <class T, class = void>
struct has_joint_payload : std::false_type
{
};
template <class T>
struct has_joint_payload<T, std::void_t<decltype(std::declval<T>().steering_targets)>>
: std::true_type
{
};
template <class T, class = void>
struct has_action : std::false_type
{
};
template <class T>
struct has_action<T, std::void_t<decltype(std::declval<T>().action)>> : std::true_type
{
};
static_assert(!has_action<Output>::value && !has_action<ChassisCommand>::value);
static_assert(!has_joint_payload<Output>::value && !has_joint_payload<ModeRequest>::value);
Output velocity(DriveMode mode, Twist2d target = {})
{
  Output output;
  output.command = ChassisCommand{mode, target, std::nullopt};
  return output;
}
Output request(std::uint64_t id, DriveMode mode, Twist2d entry)
{
  auto output = velocity(mode);
  output.command->mode_request = ModeRequest{id, mode, entry};
  return output;
}
VehicleState stopped(DriveMode mode = DriveMode::DualAckermann)
{
  VehicleState s;
  s.actual_mode = mode;
  s.stamp_s = 1;
  s.time_in_mode_s = 2;
  return s;
}
void test_target_and_shared_model()
{
  Config c;
  c.compute_budget_ratio = 0;
  auto input = scenario_input("straight");
  Controller controller(c);
  const auto output = controller.compute(input);
  check(
    output.command && output.command->target_velocity.vx > 0 && !output.command->mode_request,
    "planner must publish an ordinary positive body target");
  ChassisExecutor executor(c);
  const auto result = executor.update(output, input.vehicle);
  const auto & v = output.command->target_velocity;
  const auto predicted = DriveModel(c).step(input.vehicle, {v.vx, v.vy, v.wz}, c.dt_s);
  check(
    result.action == Action::Drive && result.steering_targets == predicted.steering_targets &&
      result.wheel_speed_targets == predicted.wheel_speed_targets,
    "public target must reproduce the shared prediction model");
  check(
    v.vx > predicted.state.velocity.vx, "target velocity must not be the rate-limited FK endpoint");

  for (auto mode : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
    for (double sign : {-1., 1.}) {
      auto s = stopped(mode);
      const Twist2d target = mode == DriveMode::Spin   ? Twist2d{0, 0, sign * .4}
                             : mode == DriveMode::Crab ? Twist2d{sign * .3, sign * .2, 0}
                                                       : Twist2d{sign * .3, 0, sign * .1};
      DriveModel model(c);
      s.steering_angles = model.steering_for_entry(mode, {target.vx, target.vy, target.wz}, {});
      ChassisExecutor e(c, mode);
      const auto expected = model.step(s, {target.vx, target.vy, target.wz}, c.dt_s);
      const auto actual = e.update(velocity(mode, target), s);
      check(
        expected.valid && !expected.aligning && actual.action == Action::Drive &&
          actual.wheel_speed_targets == expected.wheel_speed_targets &&
          actual.steering_targets == expected.steering_targets,
        "all modes and signed reverse targets must share prediction and "
        "execution");
    }
  }
  ChassisExecutor aligning(c, DriveMode::Crab);
  const auto aligned =
    aligning.update(velocity(DriveMode::Crab, {0, .3, 0}), stopped(DriveMode::Crab));
  check(
    !aligned.feedback.fault && aligned.action == Action::Hold &&
      aligned.wheel_speed_targets == std::array<double, 4>{} && aligned.steering_targets[0] > 0,
    "large same-mode steering change must align without drive");
}
void test_zero_absence_and_recovery()
{
  Config c;
  ChassisExecutor e(c);
  auto s = stopped();
  s.velocity.vx = .2;
  s.wheel_speeds.fill(.2);
  auto r = e.update(velocity(s.actual_mode), s);
  check(
    r.action == Action::Brake && !r.feedback.fault,
    "valid zero target must brake normally without latching");
  s = stopped();
  s.stamp_s += c.dt_s;
  r = e.update(velocity(s.actual_mode), s);
  check(r.action == Action::Hold && !r.feedback.fault, "zero target must settle normally");
  s.stamp_s += c.dt_s;
  r = e.update(Output{}, s);
  check(
    r.action == Action::SafeStop && r.feedback.fault,
    "absent authorization must cancel and latch independently of zero "
    "velocity");
  s.stamp_s += c.dt_s;
  check(
    e.update(velocity(s.actual_mode, {.2, 0, 0}), s).feedback.fault,
    "fresh velocity must not clear an execution fault");
  e.reset(s);
  check(
    !e.update(velocity(s.actual_mode, {.2, 0, 0}), s).feedback.fault,
    "verified recovery must permit a fresh body target");
}
void test_mode_request_immutability()
{
  Config c;
  ChassisExecutor e(c);
  auto s = stopped();
  const auto command = request(7, DriveMode::Crab, {0, .3, 0});
  const auto first = e.update(command, s);
  check(
    !first.feedback.fault && !first.feedback.confirmed && first.feedback.request_id == 7 &&
      first.feedback.actual_mode == DriveMode::DualAckermann,
    "explicit body request must preserve actual mode until measured "
    "acknowledgement");
  const auto frozen = first.steering_targets;
  actuate(s, first, c);
  const auto retry = e.update(command, s);
  check(
    !retry.feedback.fault && retry.steering_targets == frozen,
    "retry must retain entry geometry as measured steering advances");
  actuate(s, retry, c);
  auto changed = command;
  changed.command->mode_request->entry_velocity.vy = .2;
  check(
    e.update(changed, s).feedback.fault,
    "same ID with changed body entry intent must latch a fault");
  auto recovered = stopped();
  recovered.mode_request_id = 7;
  e.reset(recovered);
  check(
    e.update(command, recovered).feedback.fault, "reset must retain request-ID high-water mark");

  ChassisExecutor timeout(c);
  s = stopped();
  timeout.update(command, s);
  for (int tick = 1; tick < 30; ++tick) {
    s.stamp_s = 1 + tick * c.dt_s;
    const auto r = timeout.update(command, s);  // Deliberately no steering response.
    if (r.feedback.fault) {
      check(
        s.stamp_s > 1 + c.confirmation_timeout_s && tick < 29,
        "retries must not extend the original mode deadline");
      return;
    }
  }
  check(false, "unresponsive mode transition must time out");
}
void test_all_directed_mode_transitions()
{
  Config c;
  for (auto from : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
    for (auto to : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
      if (from == to) {
        continue;
      }
      ChassisExecutor e(c, from);
      auto s = stopped(from);
      s.steering_angles = DriveModel(c).steering_for_mode(from);
      const Twist2d entry = to == DriveMode::Spin   ? Twist2d{0, 0, .4}
                            : to == DriveMode::Crab ? Twist2d{.2, .2, 0}
                                                    : Twist2d{.3, 0, .1};
      const auto cmd = request(1, to, entry);
      bool confirmed = false;
      for (int tick = 0; tick < 25; ++tick) {
        const auto result = e.update(cmd, s);
        check(
          !result.feedback.fault && result.wheel_speed_targets == std::array<double, 4>{},
          "every directed mode transition must remain stopped and healthy");
        actuate(s, result, c);
        if (s.mode_confirmed && s.actual_mode == to) {
          confirmed = true;
          break;
        }
      }
      check(
        confirmed && s.mode_request_id == 1,
        "all six directed mode transitions must receive measured matching "
        "acknowledgement");
      const double age = s.time_in_mode_s;
      const auto retry = e.update(cmd, s);
      check(
        retry.action == Action::Hold && retry.feedback.confirmed &&
          retry.feedback.time_in_mode_s > age,
        "completed request retry must remain stopped without restarting "
        "mode age");
      actuate(s, retry, c);
      check(
        e.update(velocity(to, entry), s).action == Action::Drive,
        "body drive may resume after a measured stopped acknowledgement");
    }
  }
}
void test_malformed_commands_and_feedback()
{
  Config c;
  for (int variant = 0; variant < 7; ++variant) {
    auto s = stopped();
    auto cmd = velocity(s.actual_mode, {.3, 0, 0});
    if (variant == 0) {
      cmd.command->mode = DriveMode::Crab;  // Implicit mode change.
    }
    if (variant == 1) {
      cmd.command->target_velocity.vx = std::numeric_limits<double>::quiet_NaN();
    }
    if (variant == 2) {
      cmd.command->target_velocity.vx = 100;
    }
    if (variant == 3) {
      cmd = request(0, DriveMode::Crab, {0, .3, 0});
    }
    if (variant == 4) {
      cmd = request(1, DriveMode::Crab, {0, .3, 0});
      cmd.command->target_velocity.vy = .3;
    }
    if (variant == 5) {
      s.mode_confirmed = false;
    }
    if (variant == 6) {
      s.velocity.vx = .001;  // Diagnostic-valid, model-inconsistent.
    }
    ChassisExecutor e(c);
    check(e.update(cmd, s).feedback.fault, "malformed body command or feedback must fail closed");
  }
  Config small = c;
  small.compute_budget_ratio = 0;
  Controller invalid(small);
  auto input = scenario_input("straight");
  input.vehicle.velocity.vx = .001;
  const auto output = invalid.compute(input);
  check(
    !output.command && output.failure_reason == FailureReason::InconsistentFeedback,
    "public planner must preserve model-admission diagnostics and withhold "
    "authorization");
  input = scenario_input("straight");
  input.reference_path.clear();
  invalid.reset();
  check(!invalid.compute(input).command, "empty path must cancel through absent authorization");
}
CommandEnvelope envelope(
  const Output & output, const ControllerInput & input, std::uint64_t sequence)
{
  const auto t = input.vehicle.stamp_s;
  return {1, sequence, t, output, t, t, t + .025, CommandTask::capture(input)};
}
void test_queued_request_equivalent_geometry()
{
  Config c;
  c.compute_budget_ratio = 0;
  for (double sign : {-1., 1.}) {
    Controller controller(c);
    TimedExecutor executor(c, 1);
    ProfileRunner runner(c);
    ControllerInput input;
    input.vehicle = stopped();
    input.vehicle.steering_angles.fill(sign * .01);
    input.reference_path = {{0, 0, 0}, {0, sign * .2, 0}};
    auto output = controller.compute(input);
    check(output.command && output.command->mode_request, "near lateral goal must request Crab");
    const auto frozen = *output.command->mode_request;
    check(frozen.mode == DriveMode::Crab, "queued regression must enter Crab");
    // A valid execution snapshot changes the nearest equivalent mechanical
    // representation before the first request is accepted.
    input.vehicle.steering_angles.fill(-sign * .01);
    input.vehicle.stamp_s += c.dt_s;
    bool drove = false, complete = false;
    double wall = 10;
    for (std::uint64_t tick = 0; tick < 60; ++tick) {
      const double now = input.vehicle.stamp_s;
      if (output.command && output.command->mode_request) {
        const auto & retry = *output.command->mode_request;
        check(
          retry.id == frozen.id && retry.mode == frozen.mode &&
            retry.entry_velocity.vx == frozen.entry_velocity.vx &&
            retry.entry_velocity.vy == frozen.entry_velocity.vy &&
            retry.entry_velocity.wz == frozen.entry_velocity.wz,
          "queued retries must retain the complete original body intent");
      }
      CommandEnvelope packet{
        1,   tick + 1,   tick == 0 ? 1.05 : now,     output, tick == 0 ? 1 : now,
        now, now + .025, CommandTask::capture(input)};
      const auto guarded = executor.update(packet, input, now);
      check(
        guarded.timing_error == TimingError::None &&
          guarded.safety_error == ExecutionSafetyError::None && guarded.actuation &&
          !guarded.execution.feedback.fault,
        "equivalent queued entry must not fault planning or guarded execution");
      if (tick == 0) {
        check(
          sign * guarded.execution.steering_targets[0] < -1.5,
          "execution must freeze the opposite signed representation");
      }
      drove = drove || guarded.execution.action == Action::Drive;
      check(runner.install(guarded, now, wall), "queued profile must install");
      actuate_profile(input.vehicle, runner, guarded.execution.feedback, c, wall, {});
      wall += c.dt_s;
      output = controller.compute(input);
      check(output.command.has_value(), "queued equivalent entry must never cancel on timeout");
      if (output.goal_reached) {
        complete = true;
        break;
      }
    }
    check(
      drove && complete && controller.transition_phase() == TransitionPhase::Stable,
      "both signed lateral entries must hand over, drive and complete");
  }
}
void test_equivalent_acknowledgement_guards()
{
  Config c;
  c.steering_limit_rad = 3.14159265358979323846;
  for (auto mode : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
    auto s = stopped(mode == DriveMode::DualAckermann ? DriveMode::Crab : DriveMode::DualAckermann);
    ModeManager manager(c);
    const Control intent = mode == DriveMode::Spin   ? Control{0, 0, .3}
                           : mode == DriveMode::Crab ? Control{.2, .2, 0}
                                                     : Control{.3, 0, .1};
    manager.begin(mode, intent, s);
    const auto request = manager.update(s);
    for (std::size_t i = 0; i < 4; ++i) {
      const double angle = request.steering_targets[i];
      s.steering_angles[i] = angle + (angle > 0 ? -c.steering_limit_rad : c.steering_limit_rad);
    }
    const auto equivalent = s.steering_angles;
    s.stamp_s += c.dt_s;
    s.actual_mode = mode;
    s.mode_request_id = request.mode_request->id + 1;
    check(manager.update(s).action == Action::RequestMode, "equivalence cannot bypass request ID");
    s.mode_request_id = request.mode_request->id;
    s.actual_mode = mode == DriveMode::DualAckermann ? DriveMode::Crab : DriveMode::DualAckermann;
    s.stamp_s += c.dt_s;
    check(
      manager.update(s).action == Action::RequestMode, "equivalence still needs the target mode");
    s.actual_mode = mode;
    s.mode_confirmed = false;
    s.stamp_s += c.dt_s;
    check(manager.update(s).action == Action::RequestMode, "equivalence still needs confirmation");
    s.mode_confirmed = true;
    s.steering_angles[0] += s.steering_angles[0] > 0 ? -.1 : .1;
    s.stamp_s += c.dt_s;
    check(manager.update(s).action == Action::RequestMode, "a different rolling line must wait");
    s.steering_angles = equivalent;
    s.wheel_speeds.fill(.01);
    s.velocity = Kinematics(c).forward(s.wheel_speeds, s.steering_angles);
    s.stamp_s += c.dt_s;
    check(manager.update(s).action == Action::RequestMode, "equivalent moving joints must wait");
    s.wheel_speeds.fill(0);
    s.velocity = {};
    s.stamp_s += c.dt_s;
    const auto handover = manager.update(s);
    check(
      handover.action == Action::Hold && !manager.active() &&
        handover.steering_targets == equivalent,
      "all modes must acknowledge equivalent measured lines without re-steering");
  }
}
void test_guarded_public_pipeline()
{
  Config c;
  auto input = scenario_input("straight");
  TimedExecutor e(c, 1);
  auto r = e.update(envelope(velocity(DriveMode::DualAckermann, {.3, 0, 0}), input, 1), input, 1);
  check(
    r.actuation && r.execution.action == Action::Drive && !r.execution.feedback.fault,
    "guarded executor must compile and certify a real body target");
  input.vehicle.stamp_s += c.dt_s;
  r = e.update(std::nullopt, input, input.vehicle.stamp_s);
  check(
    !r.actuation && r.timing_error == TimingError::MissingCommand && r.execution.feedback.fault,
    "transport silence must not replay the previous target");

  TimedExecutor transactional(c, 1);
  input = scenario_input("straight");
  auto rejected = envelope(request(7, DriveMode::Crab, {0, .3, 0}), input, 1);
  rejected.source_task->path_id = 99;
  r = transactional.update(rejected, input, 1);
  check(
    r.safety_error == ExecutionSafetyError::TaskMismatch && r.actuation &&
      !r.execution.feedback.fault,
    "obsolete task must authorize only a separately checked stopping "
    "fallback");
  input.vehicle.stamp_s += c.dt_s;
  r = transactional.update(
    envelope(request(7, DriveMode::Crab, {.3, 0, 0}), input, 2), input, input.vehicle.stamp_s);
  check(
    r.actuation && !r.execution.feedback.fault && r.execution.feedback.request_id == 7,
    "rejected body request must not consume its ID or cache entry intent");

  TimedExecutor obstacles(c, 1);
  input = scenario_input("straight");
  const auto queued = envelope(velocity(DriveMode::DualAckermann, {.3, 0, 0}), input, 1);
  input.obstacles.push_back({0, 0, .1});
  r = obstacles.update(queued, input, 1);
  check(
    r.execution.feedback.fault && !r.actuation,
    "body compilation must not bypass current-obstacle and full-stop "
    "validation");

  TimedExecutor cancelled(c, 1);
  input = scenario_input("straight");
  auto cancellation = envelope(Output{}, input, 1);
  cancellation.source_task.reset();
  r = cancelled.update(cancellation, input, 1);
  check(
    r.execution.feedback.fault && !r.actuation &&
      r.safety_error != ExecutionSafetyError::TaskMismatch,
    "absent authorization must latch even without a matching source task");
}
void test_public_planner_state_and_budget()
{
  Config c;
  c.compute_budget_ratio = 0;
  auto input = scenario_input("straight");
  Controller original(c);
  check(original.compute(input).command.has_value(), "initial public planning must succeed");
  Controller copied(original);
  Controller assigned(c);
  assigned = original;
  original.reset();
  check(
    original.compute(input).command.has_value(), "reset must clear only original planner state");
  const auto repeated = copied.compute(input);
  check(
    !repeated.command && repeated.failure_reason == FailureReason::NonmonotonicTime &&
      !assigned.compute(input).command,
    "public controller copies must preserve independent timestamp state");

  Config bounded;
  int calls = 0;
  Controller late(bounded, nullptr, [&] {
    return PlanningBudget::Clock::time_point{} + std::chrono::milliseconds(81 * calls++);
  });
  const auto timeout = late.compute(input);
  check(
    !timeout.command && timeout.failure_reason == FailureReason::ComputeTimeout &&
      timeout.planning_stats.budget_exhausted,
    "public conversion must preserve deadline failure without authorizing "
    "a target");
}
}  // namespace
int main()
{
  try {
    test_target_and_shared_model();
    test_zero_absence_and_recovery();
    test_mode_request_immutability();
    test_all_directed_mode_transitions();
    test_queued_request_equivalent_geometry();
    test_equivalent_acknowledgement_guards();
    test_malformed_commands_and_feedback();
    test_guarded_public_pipeline();
    test_public_planner_state_and_budget();
    std::cout << "body target, actionless stopping, immutable mode and guarded "
                 "pipeline passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
