#include <iostream>
#include <stdexcept>

#include "behavior_fixture.hpp"
#include "swerve_mppi/feedback/feedback_adapter.hpp"
#include "swerve_mppi/planning/controller.hpp"
#include "swerve_mppi/safety/trajectory_validator.hpp"

using namespace swerve_mppi;
using namespace swerve_mppi::test;

namespace
{
ControllerInput straight() { return scenario_input("straight"); }

void test_feedback_pairing_and_freshness()
{
  Config config;
  FeedbackAdapter adapter(config);
  JointObservation joints;
  joints.stamp_ns = *duration_nanoseconds(1.0);
  joints.names = {"fl_steering_joint", "fr_steering_joint", "rl_steering_joint",
                  "rr_steering_joint", "fl_wheel_joint",    "fr_wheel_joint",
                  "rl_wheel_joint",    "rr_wheel_joint"};
  joints.positions.resize(8);
  joints.velocities.resize(8);
  StampedPose pose;
  pose.stamp_ns = *add_duration(joints.stamp_ns, .002);
  ModeFeedback mode;
  mode.stamp_ns = *add_duration(joints.stamp_ns, .004);
  const auto planning = *add_duration(joints.stamp_ns, .05);
  const auto paired = adapter.make(joints, pose, mode, planning);
  check(
    paired.error == SnapshotError::None && paired.state && paired.state->stamp_ns == mode.stamp_ns,
    "bounded source skew must pair without relabelling the observation to planning time");

  pose.stamp_ns = *add_duration(joints.stamp_ns, .006);
  check(
    adapter.make(joints, pose, mode, planning).error == SnapshotError::Unsynchronized,
    "source skew beyond the pairing bound must be rejected");
  pose.stamp_ns = joints.stamp_ns;
  mode.stamp_ns = joints.stamp_ns;
  check(
    adapter.make(joints, pose, mode, *add_duration(joints.stamp_ns, .151)).error ==
      SnapshotError::Stale,
    "stale paired feedback must be rejected");
  check(
    adapter.make(joints, pose, mode, joints.stamp_ns).error == SnapshotError::None,
    "equal source and planning timestamps must be admitted");
  check(
    adapter.make(joints, pose, mode, joints.stamp_ns - *duration_nanoseconds(.003)).error ==
      SnapshotError::Future,
    "feedback beyond the future tolerance must be rejected");
}

void test_command_lifecycle()
{
  Config config;
  config.compute_budget_ratio = 0;
  auto input = straight();
  input.planning_stamp_ns = *add_duration(input.vehicle.stamp_ns, .01);
  auto output = Controller(config).compute(input);
  check(
    output.command && output.command_id != 0 &&
      output.observation_stamp_ns == input.vehicle.stamp_ns &&
      output.computed_stamp_ns == input.planning_stamp_ns &&
      output.published_stamp_ns == kInvalidTimestamp &&
      output.valid_until_ns == *add_duration(input.planning_stamp_ns, config.command_lifetime_s),
    "controller output must keep observation, computation, publication and expiry distinct");
  check(!command_valid_at(output, output.computed_stamp_ns), "unpublished command is not valid");
  const auto published = *add_duration(output.computed_stamp_ns, .001);
  check(set_publication_stamp(output, published), "a bounded publication stamp must be accepted");
  check(!set_publication_stamp(output, published), "publication may be recorded only once");
  check(
    command_valid_at(output, published) && command_valid_at(output, output.valid_until_ns) &&
      !command_valid_at(output, output.valid_until_ns + 1),
    "command validity must begin at publication and end at expiry");
}

void test_clock_fault_latch_and_recovery()
{
  Config config;
  config.compute_budget_ratio = 0;
  Controller controller(config);
  auto input = straight();
  check(controller.compute(input).command.has_value(), "initial command must be authorized");
  const auto replay = controller.compute(input);
  check(
    !replay.command && replay.failure_reason == FailureReason::NonmonotonicTime,
    "timestamp replay must expose its immediate clock failure");
  input.vehicle.stamp_ns = *add_duration(input.vehicle.stamp_ns, config.model_period_s);
  input.planning_stamp_ns = input.vehicle.stamp_ns;
  check(
    controller.compute(input).failure_reason == FailureReason::FaultLatched,
    "clock fault must remain latched");
  check(controller.recover(input), "fresh coherent feedback must permit deliberate recovery");
  check(controller.compute(input).command.has_value(), "recovered controller must plan afresh");
}

Output advance_once(Controller & controller, ControllerInput & input, const Config & config)
{
  const auto output = controller.compute(input);
  NominalChassis plant(config);
  const auto targets = plant.update(output, input.vehicle);
  check(!targets.feedback.fault, "test command must satisfy the nominal chassis contract");
  actuate(input.vehicle, targets, config);
  input.planning_stamp_ns = input.vehicle.stamp_ns;
  return output;
}

void test_command_history_reconciliation()
{
  Config config;
  config.compute_budget_ratio = 0;
  {
    Controller controller(config);
    auto input = straight();
    const auto initial = input.vehicle;
    const auto issued = controller.compute(input);
    check(issued.command.has_value(), "asynchronous history test requires a command");
    const auto application = *add_duration(issued.computed_stamp_ns, .01);
    DriveModel model(config);
    const auto before = model.step(initial, {}, .01);
    const auto & velocity = issued.command->target_velocity;
    const auto after =
      model.step(before.state, {velocity.vx, velocity.vy, velocity.wz}, .09, before.prediction);
    check(before.valid && after.valid, "asynchronous reference prediction must be valid");
    input.vehicle = after.state;
    input.planning_stamp_ns = input.vehicle.stamp_ns;
    input.previous_command_application =
      CommandApplication{issued.command_id, issued.computed_stamp_ns, application, application};
    check(
      controller.compute(input).prediction_history == PredictionHistoryStatus::Synchronized,
      "application after observation must propagate the prior held command before the new one");
  }
  {
    Controller controller(config);
    auto input = straight();
    const auto issued = advance_once(controller, input, config);
    input.previous_command_application = CommandApplication{
      issued.command_id, issued.computed_stamp_ns, issued.computed_stamp_ns,
      issued.computed_stamp_ns};
    const auto next = controller.compute(input);
    check(
      next.command && next.prediction_history == PredictionHistoryStatus::Synchronized,
      "exact reported application and matching joints must synchronize hidden model state");
  }
  {
    Controller controller(config);
    auto input = straight();
    advance_once(controller, input, config);
    check(
      controller.compute(input).prediction_history == PredictionHistoryStatus::MissingApplication,
      "missing application evidence must force an explicit cold prediction");
  }
  {
    Controller controller(config);
    auto input = straight();
    const auto issued = advance_once(controller, input, config);
    input.previous_command_application = CommandApplication{
      issued.command_id, issued.computed_stamp_ns, issued.computed_stamp_ns,
      *add_duration(issued.computed_stamp_ns, .001)};
    const auto uncertain = controller.compute(input);
    check(
      uncertain.command &&
        uncertain.prediction_history == PredictionHistoryStatus::ApplicationUncertain,
      "bounded but nonzero application uncertainty must remain explicit and cold-seeded");
  }
  {
    Controller controller(config);
    auto input = straight();
    const auto issued = advance_once(controller, input, config);
    input.previous_command_application = CommandApplication{
      issued.command_id + 1, issued.computed_stamp_ns, issued.computed_stamp_ns,
      issued.computed_stamp_ns};
    const auto invalid = controller.compute(input);
    check(
      !invalid.command && invalid.failure_reason == FailureReason::ClockFault &&
        invalid.prediction_history == PredictionHistoryStatus::InvalidApplication,
      "wrong command identity must latch a clock/application fault");
  }
  {
    Controller controller(config);
    auto input = straight();
    input.reference_path = {{0, 0, 0}};
    const auto issued = advance_once(controller, input, config);
    input.vehicle.steering_angles.fill(.2);
    input.previous_command_application = CommandApplication{
      issued.command_id, issued.computed_stamp_ns, issued.computed_stamp_ns,
      issued.computed_stamp_ns};
    const auto diverged = controller.compute(input);
    check(
      !diverged.command && diverged.failure_reason == FailureReason::InconsistentFeedback &&
        diverged.prediction_history == PredictionHistoryStatus::Diverged,
      "measured joints outside the history envelope must latch feedback divergence");
  }
}

void test_independent_motion_policy()
{
  Config config;
  config.compute_budget_ratio = 0;
  auto input = straight();
  input.motion_policy = MotionPolicy::RequireIndependent;
  Controller missing(config);
  const auto rejected = missing.compute(input);
  check(
    !rejected.command && rejected.failure_reason == FailureReason::MotionUncertainty &&
      rejected.motion_status == MotionStatus::Missing,
    "required independent motion evidence cannot be omitted");

  input.motion_observation = MotionObservation{
    input.vehicle.velocity, input.vehicle.stamp_ns, MotionSource::IndependentBody};
  const auto accepted = Controller(config).compute(input);
  check(
    accepted.command && accepted.motion_status == MotionStatus::NominalAgreement,
    "fresh independent motion evidence must authorize nominal planning");
}

void test_motion_uncertainty_expands_clearance()
{
  Config config;
  auto input = straight();
  input.obstacles = {{.552, 0, 0}};
  Trajectory trace;
  trace.valid = true;
  trace.poses = {input.vehicle.pose, input.vehicle.pose};
  trace.sweep_margins_m = {0};
  trace.final_state = input.vehicle;
  TrajectoryValidator validator(config);
  check(
    validator.check(input, trace) == TrajectoryStatus::Valid,
    "nominal stationary trace must retain its measured clearance");
  MotionObservation motion{{.02, 0, 0}, input.vehicle.stamp_ns, MotionSource::IndependentBody};
  motion.linear_error_bound_mps = .01;
  input.motion_observation = motion;
  check(
    validator.check(input, trace) == TrajectoryStatus::Collision,
    "bounded independent residual must expand future hard-clearance margins");
}
}  // namespace

int main()
{
  try {
    test_feedback_pairing_and_freshness();
    test_command_lifecycle();
    test_clock_fault_latch_and_recovery();
    test_command_history_reconciliation();
    test_independent_motion_policy();
    test_motion_uncertainty_expands_clearance();
    std::cout << "Timing and lifecycle regressions passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
