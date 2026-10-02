#include "behavior_fixture.hpp"
#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/timing.hpp"
#include <iostream>

using namespace swerve_mppi;
using namespace swerve_mppi::test;
namespace {
ControllerInput straight() {
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 0}, {2, 0, 0}};
  return in;
}
Config deterministic() {
  Config c;
  c.noise_v_mps = c.noise_w_radps = 0;
  c.minimum_mode_dwell_s = 100;
  c.samples_per_branch = 8;
  return c;
}
void test_trajectory_anchor() {
  Config c;
  TrajectoryValidator validator(c);
  auto in = straight();
  Trajectory trace;
  trace.valid = true;
  trace.poses = {{0, 0, 0}, {1, 0, 0}};
  check(validator.check(in, trace) == TrajectoryStatus::Valid,
        "anchored caller-provided piecewise linear traces must remain valid");
  for (const Pose2d pose : {Pose2d{2, 0, 0}, Pose2d{0, 2, 0}, Pose2d{0, 0, .1}}) {
    trace.poses[0] = pose;
    check(validator.check(in, trace) == TrajectoryStatus::Invalid,
          "translated or rotated trajectory anchors must be rejected");
  }
  trace.poses[0] = {};
  in.vehicle.pose.x = .1;
  check(validator.check(in, trace) == TrajectoryStatus::Invalid,
        "cached traces must be rejected after feedback advances");
  in.vehicle.pose = {0, 0, -std::numeric_limits<double>::max()};
  trace.poses = {{0, 0, std::numeric_limits<double>::max()}};
  check(validator.check(in, trace) == TrajectoryStatus::Invalid,
        "overflowing yaw differences must fail closed");
  in.vehicle.pose = {};
  trace.poses = {{0, 0, 2 * std::acos(-1.0)}};
  check(validator.check(in, trace) == TrajectoryStatus::Valid,
        "equivalent wrapped yaw and a single stationary pose must be accepted");
  const double radius = c.robot_radius_m + c.collision_margin_m + .1;
  in.obstacles = {{-radius + 1e-10, 0, .1}};
  trace.poses = {{5e-10, 0, 0}};
  check(validator.check(in, trace) == TrajectoryStatus::Collision,
        "anchor tolerance must never omit the exact measured colliding footprint");
  trace.poses = {{2, 0, 0}, {3, 0, 0}};
  in.obstacles = {{0, 0, .1}};
  check(validator.check(in, trace) == TrajectoryStatus::Invalid,
        "the reviewed remote safe trace must not validate a colliding current vehicle");
  trace.poses = {{0, 0, 0}};
  check(validator.check(in, trace) == TrajectoryStatus::Collision,
        "anchored single-pose holds must check current collision");
}
void test_validator_configuration_contract() {
  const Config c;
  for (int field = 0; field < 4; ++field) {
    Config different = c;
    if (field == 0)
      different.robot_radius_m = .4;
    if (field == 1)
      different.collision_margin_m = .01;
    if (field == 2)
      different.steering_limit_rad = 2;
    if (field == 3)
      different.max_wheel_speed_mps = 3;
    const auto validator = std::make_shared<TrajectoryValidator>(different);
    for (int consumer = 0; consumer < 3; ++consumer) {
      bool rejected = false;
      try {
        if (consumer == 0) {
          Controller controller(c, validator);
        }
        if (consumer == 1) {
          Optimizer optimizer(c, validator);
        }
        if (consumer == 2) {
          CriticManager critics(c, validator);
        }
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      check(rejected, "all validator consumers must reject a different safety configuration");
    }
  }
  Config planning = c;
  planning.random_seed = 7;
  planning.goal_weight = 2;
  auto validator = std::make_shared<TrajectoryValidator>(planning);
  Controller controller(c, validator);
  auto input = straight();
  input.reference_path.back().x = .2;
  input.obstacles = {{.555, 0, .05}};
  check(controller.compute(input).action == Action::SafeStop,
        "compatible injected validator must enforce the controller footprint during capture");
  check(!std::isfinite(Optimizer(c, validator).optimize(input, {}).cost),
        "compatible injected validator must enforce the same footprint during tracking");
}
void test_temporary_failure_stops_and_recovers() {
  const auto c = deterministic();
  Controller controller(c);
  ModeExecutor executor(c);
  auto in = straight();
  in.vehicle.velocity.vx = .2;
  in.vehicle.wheel_speeds.fill(.2);
  in.obstacles = {{.65, 0, .05}};
  const auto blocked = controller.compute(in);
  check(blocked.action == Action::Brake &&
            blocked.failure_reason == FailureReason::NoFeasiblePlan &&
            blocked.control_policy == ControlPolicy::Blocked &&
            blocked.navigation_status == NavigationStatus::Waiting &&
            blocked.steering_targets == in.vehicle.steering_angles && !blocked.goal_reached,
        "temporary infeasibility must emit checked braking with waiting diagnostics");
  const auto stopped = executor.update(blocked, in.vehicle);
  check(!stopped.feedback.fault && stopped.feedback.actual_mode == DriveMode::DualAckermann,
        "checked planning stop must neither latch fault nor erase actual mode");
  actuate(in.vehicle, stopped, c);
  in.obstacles.clear();
  const auto resumed = controller.compute(in);
  check(resumed.action == Action::Drive && resumed.failure_reason == FailureReason::None &&
            !executor.update(resumed, in.vehicle).feedback.fault,
        "a cleared temporary obstacle must allow fresh planning without reset");
}
void test_unsafe_stopping_latches_fault() {
  const auto c = deterministic();
  Controller controller(c);
  ModeExecutor executor(c);
  auto in = straight();
  in.vehicle.velocity.vx = .3;
  in.vehicle.wheel_speeds.fill(.3);
  // Current footprint is clear, but even the predicted braking path hits this obstacle.
  in.obstacles = {{.605, 0, .05}};
  const auto command = controller.compute(in);
  check(command.action == Action::SafeStop &&
            command.failure_reason == FailureReason::UnsafeStoppingTrajectory,
        "a rejected stopping trajectory cannot become a recoverable planning stop");
  check(executor.update(command, in.vehicle).feedback.fault,
        "unsafe stopping must latch the execution fault");
  in.obstacles.clear();
  in.vehicle.stamp_s += c.dt_s;
  check(executor.update(controller.compute(in), in.vehicle).feedback.fault,
        "removing an obstacle must not automatically recover a latched fault");
}
void test_capture_checks_one_command_then_stop() {
  const auto c = deterministic();
  Controller controller(c);
  ModeExecutor executor(c);
  auto in = straight();
  in.reference_path.back().x = .2;
  in.obstacles = {{.9, 0, .05}};
  bool completed = false;
  for (int tick = 0; tick < 50; ++tick) {
    const auto command = controller.compute(in);
    check(command.action != Action::SafeStop && command.failure_reason == FailureReason::None,
          "capture must not extrapolate a constant command past a safe goal into an obstacle");
    const auto execution = executor.update(command, in.vehicle);
    check(!execution.feedback.fault, "bounded capture must execute without recovery");
    if (command.goal_reached) {
      completed = true;
      break;
    }
    actuate(in.vehicle, execution, c);
  }
  check(completed, "capture before an obstacle must complete with measured stopped dwell");
}
void test_terminal_brakes_share_stopping_validation() {
  for (int route = 0; route < 5; ++route) {
    Config c;
    c.max_wheel_accel_mps2 = .2;
    Controller controller(c);
    ModeExecutor executor(c);
    auto in = straight();
    in.reference_path = {{0, 0, route == 1 || route == 3 ? 1.0 : 0.0}};
    if (route == 2)
      in.reference_path = {{0, 0, 0}, {0, .2, 0}};
    if (route == 3)
      in.vehicle.time_in_mode_s = 0;
    if (route == 4) {
      for (int tick = 0; tick < 5; ++tick) {
        const auto command = controller.compute(in);
        check(!executor.update(command, in.vehicle).feedback.fault,
              "safe settling must remain healthy before external motion");
        in.vehicle.stamp_s += c.dt_s;
        if (tick == 4)
          check(command.goal_reached, "test must establish completion before external motion");
      }
    }
    in.vehicle.velocity.vx = .3;
    in.vehicle.wheel_speeds.fill(.3);
    in.obstacles = {{.605, 0, .05}};
    const auto command = controller.compute(in);
    check(command.action == Action::SafeStop &&
              command.failure_reason == FailureReason::UnsafeStoppingTrajectory &&
              command.control_policy == ControlPolicy::Fault && !command.goal_reached,
          "settling, yaw, translation, dwell and completed-task braking must reject unsafe stops");
    check(executor.update(command, in.vehicle).feedback.fault,
          "terminal stop rejection must latch the same execution fault as tracking");
  }
}
void test_safe_terminal_braking_retains_navigation_status() {
  Config c;
  Controller controller(c);
  ModeExecutor executor(c);
  auto in = straight();
  in.reference_path = {{0, 0, 1}};
  in.vehicle.velocity.vx = .1;
  in.vehicle.wheel_speeds.fill(.1);
  const auto command = controller.compute(in);
  check(command.action == Action::Brake && command.failure_reason == FailureReason::None &&
            command.control_policy == ControlPolicy::Capture &&
            command.navigation_status == NavigationStatus::AligningGoal &&
            !executor.update(command, in.vehicle).feedback.fault,
        "a safe normal terminal brake must not become a blocked planning stop");
}
void test_drive_requires_complete_stopping_continuation() {
  auto c = deterministic();
  c.safety_reduction_attempts = 0;
  c.horizon_steps = 2;
  c.max_linear_decel_mps2 = .1;
  for (double goal : {2.0, .2}) {
    Controller controller(c);
    ModeExecutor executor(c);
    auto in = straight();
    in.reference_path.back().x = goal;
    in.obstacles = {{.64, 0, .05}};
    const auto blocked = controller.compute(in);
    check(blocked.action == Action::Hold &&
              blocked.failure_reason == FailureReason::NoFeasiblePlan &&
              !executor.update(blocked, in.vehicle).feedback.fault,
          "tracking and capture must reject a first Drive whose full stop hits an obstacle");
    in.obstacles.clear();
    in.vehicle.stamp_s += c.dt_s;
    const auto safe = controller.compute(in);
    check(safe.action == Action::Drive && !executor.update(safe, in.vehicle).feedback.fault,
          "a short MPPI horizon must allow a safe stop longer than its prediction horizon");
  }
  c.stopping_horizon_steps = 2;
  Controller limited(c);
  auto in = straight();
  check(limited.compute(in).action == Action::Hold,
        "an exhausted stopping budget must reject Drive, even with no obstacles");
  in.vehicle.velocity.vx = .2;
  in.vehicle.wheel_speeds.fill(.2);
  in.vehicle.stamp_s += c.dt_s;
  const auto fault = limited.compute(in);
  check(fault.action == Action::SafeStop &&
            fault.failure_reason == FailureReason::UnsafeStoppingTrajectory,
        "an incomplete current stop must fail closed rather than pretending to be stopped");
}
void test_first_drive_deceleration_matches_execution() {
  Config c;
  c.collision_margin_m = 0;
  c.goal_position_tolerance_m = .001;
  auto in = straight();
  in.vehicle.velocity.vx = .1;
  in.vehicle.wheel_speeds.fill(.1);
  in.reference_path.back().x = .04;
  in.obstacles = {{.558, 0, .05}};
  Trajectory unsafe;
  RolloutEngine(c).generate_continuation(in.vehicle, {}, {.048, 0, 0}, unsafe);
  // Independent full-period Drive plus analytic full brake: mean speed * dt,
  // then v^2/(2*a). The old fastest-ramp prediction incorrectly passed 8 mm.
  const double expected = (.1 + .048) * c.dt_s / 2 + .048 * .048 / (2 * c.max_linear_decel_mps2);
  check(unsafe.valid && std::abs(unsafe.final_state.pose.x - expected) < 1e-10 &&
            TrajectoryValidator(c).check(in, unsafe) == TrajectoryStatus::Collision,
        "decelerating first Drive must include its full-period displacement");
  Controller controller(c);
  ModeExecutor executor(c);
  const auto output = controller.compute(in);
  check(output.action == Action::Drive && output.safety_reductions > 0 &&
            output.body_command.vx < .048,
        "a rejected capture may resume only with a checked reduced intent");
  auto result = executor.update(output, in.vehicle);
  check(!result.feedback.fault, "reduced Drive must satisfy the same executor contract");
  actuate(in.vehicle, result, c);
  Output brake;
  brake.action = Action::Brake;
  brake.requested_mode = in.vehicle.actual_mode;
  actuate(in.vehicle, executor.update(brake, in.vehicle), c);
  check(in.vehicle.pose.x < .008 && is_stopped(in.vehicle, c),
        "independent first Drive and Brake must stay outside the obstacle boundary");
}
void test_stopping_budget_reduction_restores_progress() {
  auto c = deterministic();
  c.max_linear_decel_mps2 = .001;
  c.samples_per_branch = c.iterations = 1;
  auto in = straight();
  in.reference_path.back().x = 1;
  Controller controller(c);
  ModeExecutor executor(c);
  const auto output = controller.compute(in);
  check(output.action == Action::Drive && output.safety_reductions > 0 &&
            output.safety_reductions <= c.safety_reduction_attempts,
        "bounded reduction must find a safe slow Drive within the stopping budget");
  auto result = executor.update(output, in.vehicle);
  check(!result.feedback.fault, "reduced tracking output must execute healthily");
  actuate(in.vehicle, result, c);
  check(in.vehicle.pose.x > 0, "a safe reduced intent must make measured progress");
  c.safety_reduction_attempts = 0;
  Controller disabled(c);
  in = straight();
  check(disabled.compute(in).action == Action::Hold,
        "zero reduction budget retains checked waiting semantics");
  c.safety_reduction_attempts = 17;
  bool rejected = false;
  try {
    validate(c);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "reduction work must have a bounded configurable maximum");
}
void test_overspeed_feedback_uses_checked_braking() {
  auto c = deterministic();
  auto in = straight();
  in.vehicle.velocity.vx = 1;
  in.vehicle.wheel_speeds.fill(1);
  const auto output = Controller(c).compute(in);
  check(output.action == Action::Brake &&
            !ModeExecutor(c).update(output, in.vehicle).feedback.fault,
        "measured overspeed must recover by checked Brake instead of an overspeed Drive");
}
void test_zero_intent_braking_executes_with_measured_residual() {
  auto c = deterministic();
  c.path_lookahead_m = .1;
  c.max_wheel_accel_mps2 = .1;
  c.samples_per_branch = c.iterations = 1;
  Controller controller(c);
  ModeExecutor executor(c);
  auto in = straight();
  in.vehicle.wheel_speeds = {.3, .24, .24, .3};
  in.vehicle.velocity = Kinematics(c).forward(in.vehicle.wheel_speeds, in.vehicle.steering_angles);
  in.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
  const auto command = controller.compute(in);
  check(command.action == Action::Brake && command.failure_reason == FailureReason::None &&
            command.steering_targets == in.vehicle.steering_angles &&
            command.wheel_speed_targets == std::array<double, 4>{} &&
            command.body_command.vx == 0 && !executor.update(command, in.vehicle).feedback.fault,
        "zero intent with a measured rolling residual must remain a healthy braking action");
}
class RejectAll final : public TrajectoryConstraint {
public:
  bool allows(const ControllerInput &, const Trajectory &) const override { return false; }
};
void test_stationary_terminal_hold_checks_constraints() {
  Config c;
  auto validator = std::make_shared<TrajectoryValidator>(c);
  validator->add(std::make_shared<RejectAll>());
  // Both yaw-settling and waiting for minimum mode dwell used to return early.
  for (double yaw : {0.0, 1.0}) {
    Controller controller(c, validator);
    auto in = straight();
    in.reference_path = {{0, 0, yaw}};
    in.vehicle.time_in_mode_s = 0;
    const auto command = controller.compute(in);
    check(command.action == Action::SafeStop &&
              command.failure_reason == FailureReason::UnsafeStoppingTrajectory,
          "stationary terminal holds must enforce injected stopping constraints");
  }
}
void test_pending_request_rechecks_fresh_stopping_constraints() {
  Config c;
  c.max_wheel_accel_mps2 = .2;
  Controller controller(c);
  ModeExecutor executor(c);
  auto in = straight();
  in.reference_path = {{0, 0, 1}};
  const auto request = controller.compute(in);
  check(request.action == Action::RequestMode, "test must commit a Spin request");
  actuate(in.vehicle, executor.update(request, in.vehicle), c);
  check(!in.vehicle.mode_confirmed, "test must reach an unconfirmed measured transition");
  // External motion while the handshake is pending needs braking at the current
  // measured angles. Its stop must not assume that a mode has been confirmed.
  in.vehicle.wheel_speeds.fill(.3);
  in.vehicle.velocity = Kinematics(c).forward(in.vehicle.wheel_speeds, in.vehicle.steering_angles);
  const auto retry = controller.compute(in);
  check(retry.action == Action::RequestMode && retry.mode_request &&
            retry.mode_request->id == request.mode_request->id &&
            retry.mode_request->steering_targets == request.mode_request->steering_targets,
        "safe braking during unconfirmed feedback must retain the committed request");
  const auto braking = executor.update(retry, in.vehicle);
  check(braking.action == Action::Brake && !braking.feedback.fault && !braking.feedback.confirmed,
        "safe pending-request braking cannot grant confirmation or latch a fault");
  actuate(in.vehicle, braking, c);
  in.obstacles = {{in.vehicle.pose.x + .605, in.vehicle.pose.y, .05}};
  const auto rejected = controller.compute(in);
  check(rejected.action == Action::SafeStop &&
            rejected.failure_reason == FailureReason::UnsafeStoppingTrajectory &&
            !rejected.mode_request && rejected.steering_targets == in.vehicle.steering_angles,
        "pending transitions must recheck fresh obstacles and cancel unsafe requests");
  check(executor.update(rejected, in.vehicle).feedback.fault,
        "unsafe pending transition must latch fault without completing the requested mode");
}
// The far obstacle is a context marker, never a geometric collision. A fresh
// context can prohibit further steering even when a stationary brake is safe.
class SteeringLock final : public TrajectoryConstraint {
public:
  bool allows(const ControllerInput &in, const Trajectory &trace) const override {
    return in.obstacles.empty() ||
           trace.final_state.steering_angles == in.vehicle.steering_angles;
  }
};
void test_pending_request_rechecks_actual_steering() {
  for (bool guarded : {false, true}) {
    for (double residual : {0.0, .004}) {
      Config c;
      auto validator = std::make_shared<TrajectoryValidator>(c);
      validator->add(std::make_shared<SteeringLock>());
      Controller controller(c, validator);
      ModeExecutor direct(c);
      TimedExecutor timed(c, 10, DriveMode::DualAckermann, {}, validator);
      auto in = straight();
      in.reference_path = {{0, 0, 1}};
      std::uint64_t sequence = 0;
      auto execute = [&](const Output &command) {
        if (!guarded)
          return direct.update(command, in.vehicle);
        const double now = in.vehicle.stamp_s;
        return timed.update(CommandEnvelope{10, ++sequence, now, command, now, now,
                            now + .025, CommandTask::capture(in)}, in, now).execution;
      };
      const auto first = controller.compute(in);
      check(first.action == Action::RequestMode && first.mode_request,
            "test must begin an explicit Spin request");
      actuate(in.vehicle, execute(first), c);
      check(!in.vehicle.mode_confirmed && in.vehicle.steering_angles[0] != 0,
            "test must reach measured steering in an unfinished transition");
      const auto retry = controller.compute(in);
      check(retry.action == Action::RequestMode && retry.mode_request &&
                retry.mode_request->id == first.mode_request->id &&
                retry.mode_request->steering_targets == first.mode_request->steering_targets,
            "safe retries must preserve the immutable request payload");
      const auto continued = execute(retry);
      check(!continued.feedback.fault && !continued.feedback.confirmed,
            "a safe retry must continue without granting mode confirmation");
      actuate(in.vehicle, continued, c);
      in.vehicle.wheel_speeds.fill(residual);
      in.vehicle.velocity = Kinematics(c).forward(in.vehicle.wheel_speeds,
                                                 in.vehicle.steering_angles);
      in.obstacles = {{100, 100, .05}};
      Trajectory brake;
      RolloutEngine(c).generate_stop(in.vehicle, brake);
      check(validator->check(in, brake) == TrajectoryStatus::Valid,
            "the reviewed steering constraint must still allow the stationary brake");
      const auto rejected = controller.compute(in);
      check(rejected.action == Action::SafeStop &&
                rejected.failure_reason == FailureReason::UnsafeStoppingTrajectory &&
                !rejected.mode_request && !rejected.goal_reached &&
                rejected.steering_targets == in.vehicle.steering_angles,
            "pending requests must validate actual steering, including residual braking");
      const auto stopped = execute(rejected);
      check(stopped.feedback.fault && stopped.action == Action::SafeStop &&
                stopped.steering_targets == in.vehicle.steering_angles &&
                stopped.feedback.actual_mode == DriveMode::DualAckermann &&
                stopped.feedback.request_id == first.mode_request->id,
            "both execution paths must latch fault before forbidden steering continues");
    }
  }
}
void test_guarded_retry_rechecks_changed_steering_constraint() {
  Config c;
  auto validator = std::make_shared<TrajectoryValidator>(c);
  validator->add(std::make_shared<SteeringLock>());
  Controller controller(c, validator);
  TimedExecutor executor(c, 10, DriveMode::DualAckermann, {}, validator);
  auto in = straight();
  in.reference_path = {{0, 0, 1}};
  const auto source = in;
  const auto request = controller.compute(source);
  const auto first = executor.update(CommandEnvelope{10, 1, 1, request, 1, 1, 1.025,
                                    CommandTask::capture(source)}, in, 1);
  check(first.actuation.has_value(), "safe initial transition must have a checked profile");
  actuate(in.vehicle, first.execution, c);
  in.obstacles = {{100, 100, .05}};
  // A queued immutable retry was planned before the steering lock appeared.
  const double now = in.vehicle.stamp_s;
  const auto rejected = executor.update(CommandEnvelope{10, 2, now, request, 1, now, now + .025,
                                       CommandTask::capture(source)}, in, now);
  check(rejected.timing_error == TimingError::None &&
            rejected.safety_error == ExecutionSafetyError::UnsafeStoppingTrajectory &&
            rejected.rejected_status == TrajectoryStatus::Rejected &&
            !rejected.actuation && rejected.execution.feedback.fault &&
            rejected.execution.steering_targets == in.vehicle.steering_angles,
        "execution must independently reject forbidden alignment in a committed retry/fallback");
}
void test_stop_rollout_preserves_unconfirmed_feedback() {
  Config c;
  auto in = straight();
  in.vehicle.mode_confirmed = false;
  in.vehicle.mode_request_id = 17;
  in.vehicle.velocity.vx = .3;
  in.vehicle.wheel_speeds.fill(.3);
  RolloutEngine engine(c);
  Trajectory stop;
  engine.generate_stop(in.vehicle, stop);
  check(stop.valid && stop.poses.size() > 1 && stop.poses.size() <= c.stopping_horizon_steps + 1 &&
            is_stopped(stop.final_state, c) && !stop.final_state.mode_confirmed &&
            stop.final_state.mode_request_id == 17 &&
            stop.final_state.actual_mode == in.vehicle.actual_mode &&
            stop.final_state.steering_angles == in.vehicle.steering_angles,
        "stopping prediction must preserve measured mode, request and confirmation state");
  check(!engine.generate(in.vehicle, {}, std::vector<Control>(c.horizon_steps)).valid,
        "unconfirmed feedback must still reject ordinary driving rollouts");
  in.vehicle.stamp_s = -1;
  engine.generate_stop(in.vehicle, stop);
  check(!stop.valid && stop.poses.empty(), "a failed reused stopping trace cannot retain validity");
}
class TranslationLimit final : public TrajectoryConstraint {
public:
  bool allows(const ControllerInput &input, const Trajectory &trajectory) const override {
    // A fresh world constraint activates after the first measured tick.
    if (input.vehicle.stamp_s < 1.05)
      return true;
    for (const auto &pose : trajectory.poses)
      if (std::hypot(pose.x, pose.y) > .0005)
        return false;
    return true;
  }
};
void test_shared_hard_constraints() {
  auto c = deterministic();
  c.safety_reduction_attempts = 0;
  auto validator = std::make_shared<TrajectoryValidator>(c);
  validator->add(std::make_shared<TranslationLimit>());
  for (int route = 0; route < 3; ++route) {
    Controller controller(c, validator);
    auto in = straight();
    if (route == 1)
      in.reference_path.back().x = .2;
    if (route == 2) {
      in.vehicle.actual_mode = DriveMode::Crab;
      in.reference_path = {{0, 0, 0}, {0, .1, 0}};
      check(controller.compute(in).action == Action::Hold,
            "unrestricted capture must begin committed alignment");
    }
    in.vehicle.stamp_s += c.dt_s;
    const auto rejected = controller.compute(in);
    check(rejected.action == Action::Hold &&
              rejected.failure_reason == FailureReason::NoFeasiblePlan,
          "tracking, capture and alignment must share injected hard constraints");
  }
  auto in = straight();
  in.vehicle.stamp_s = 1.1;
  check(!std::isfinite(Optimizer(c, validator).optimize(in, {}).cost),
        "standalone optimizer must also enforce the injected hard validator");
  Trajectory invalid;
  check(validator->check(in, invalid) == TrajectoryStatus::Invalid,
        "invalid predictions must be distinguishable from world constraints");
  auto trajectory =
      RolloutEngine(c).generate(in.vehicle, {}, std::vector<Control>(c.horizon_steps));
  in.obstacles = {{0, 0, .1}};
  check(validator->check(in, trajectory) == TrajectoryStatus::Collision,
        "current footprint collision must remain a hard rejection");
}

void test_residual_hold_obstacle() {
  Config c;
  c.dt_s = .5;
  c.stopped_linear_mps = .25;
  c.stopped_wheel_speed_mps = .21;
  ControllerInput in;
  in.vehicle.actual_mode = DriveMode::Crab;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.vehicle.velocity.vx = .2;
  in.vehicle.wheel_speeds.fill(.2);
  in.reference_path = {{0, 0, 0}, {0, .2, 0}};
  // Independent old simultaneous-brake/steer integral placed this footprint
  // 0.2 mm inside the obstacle; the phased brake ends at (0.02, 0).
  const double radius = c.robot_radius_m + c.collision_margin_m + .05 - .0002;
  in.obstacles = {{.0195821866951 - .1 * radius, .00329207621933 + std::sqrt(.99) * radius, .05}};
  const auto predicted = DriveModel(c).step(in.vehicle, {0, .3, 0}, c.dt_s);
  check(predicted.valid && std::abs(predicted.state.pose.x - .02) < 1e-9 &&
            std::abs(predicted.state.pose.y) < 1e-9,
        "Hold prediction must retain the complete residual braking distance without rolling "
        "steering");
  const auto output = Controller(c).compute(in);
  ModeExecutor executor(c, DriveMode::Crab);
  const auto execution = executor.update(output, in.vehicle);
  auto actual = in.vehicle;
  test::actuate(actual, execution, c);
  check(test::measured_clearance(in.vehicle.pose, actual.pose, in.obstacles, c) > 0,
        "the reviewed residual-Hold obstacle must remain clear under independent execution");
}

} // namespace
int main() {
  try {
    test_trajectory_anchor();
    test_residual_hold_obstacle();
    test_drive_requires_complete_stopping_continuation();
    test_first_drive_deceleration_matches_execution();
    test_stopping_budget_reduction_restores_progress();
    test_overspeed_feedback_uses_checked_braking();
    test_zero_intent_braking_executes_with_measured_residual();
    test_temporary_failure_stops_and_recovers();
    test_unsafe_stopping_latches_fault();
    test_capture_checks_one_command_then_stop();
    test_terminal_brakes_share_stopping_validation();
    test_safe_terminal_braking_retains_navigation_status();
    test_stationary_terminal_hold_checks_constraints();
    test_pending_request_rechecks_fresh_stopping_constraints();
    test_pending_request_rechecks_actual_steering();
    test_guarded_retry_rechecks_changed_steering_constraint();
    test_stop_rollout_preserves_unconfirmed_feedback();
    test_shared_hard_constraints();
    test_validator_configuration_contract();
    std::cout << "Safety regressions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
