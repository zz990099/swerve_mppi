#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/executor.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace swerve_mppi;
namespace {
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
Control intent(DriveMode mode) {
  switch (mode) {
  case DriveMode::DualAckermann:
    return {.4, 0, .2};
  case DriveMode::Spin:
    return {0, 0, .5};
  case DriveMode::Crab:
    return {0, .4, 0};
  }
  return {};
}
void feedback(VehicleState &s, const ModeFeedback &f) {
  s.actual_mode = f.actual_mode;
  s.mode_confirmed = f.confirmed;
  s.mode_fault = f.fault;
  s.mode_request_id = f.request_id;
  s.time_in_mode_s = f.time_in_mode_s;
}
// A simple independent actuator fixture: no DriveModel/TransitionModel stepping.
// It limits encoder acceleration and steering rate, then derives measured twist.
void actuate(VehicleState &s, const ExecutionResult &r, const Config &c) {
  const bool stopped = is_stopped(s, c);
  for (std::size_t i = 0; i < 4; ++i) {
    if (stopped || r.action == Action::Drive)
      s.steering_angles[i] +=
          std::clamp(r.steering_targets[i] - s.steering_angles[i], -c.max_steer_rate_radps * c.dt_s,
                     c.max_steer_rate_radps * c.dt_s);
    s.wheel_speeds[i] +=
        std::clamp(r.wheel_speed_targets[i] - s.wheel_speeds[i], -c.max_wheel_accel_mps2 * c.dt_s,
                   c.max_wheel_accel_mps2 * c.dt_s);
  }
  s.velocity = Kinematics(c).forward(s.wheel_speeds, s.steering_angles);
  s.pose.x +=
      (std::cos(s.pose.yaw) * s.velocity.vx - std::sin(s.pose.yaw) * s.velocity.vy) * c.dt_s;
  s.pose.y +=
      (std::sin(s.pose.yaw) * s.velocity.vx + std::cos(s.pose.yaw) * s.velocity.vy) * c.dt_s;
  s.pose.yaw = wrap_angle(s.pose.yaw + s.velocity.wz * c.dt_s);
  feedback(s, r.feedback);
  s.stamp_s += c.dt_s;
}
Output request(std::uint64_t id, DriveMode mode, const Config &c) {
  Output out;
  out.action = Action::RequestMode;
  out.requested_mode = mode;
  out.mode_request =
      ModeRequest{id, mode, DriveModel(c).steering_for_entry(mode, intent(mode), {})};
  out.steering_targets = out.mode_request->steering_targets;
  return out;
}
void test_all_directed_transitions() {
  Config c;
  for (auto from : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
    for (auto to : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
      if (from == to)
        continue;
      VehicleState s;
      s.actual_mode = from;
      s.stamp_s = 1;
      s.time_in_mode_s = 2;
      auto initial = Kinematics(c).inverse(intent(from), {});
      s.steering_angles = initial.angles;
      s.wheel_speeds = initial.speeds;
      s.velocity = Kinematics(c).forward(s.wheel_speeds, s.steering_angles);
      ModeManager manager(c);
      ModeExecutor executor(c, from);
      manager.begin(to, intent(to), s);
      std::optional<ModeRequest> frozen;
      bool finished = false;
      for (int tick = 0; tick < 40; ++tick) {
        auto command = manager.update(s);
        if (command.mode_request) {
          if (frozen)
            check(command.mode_request->id == frozen->id &&
                      command.mode_request->steering_targets == frozen->steering_targets,
                  "request payload must remain frozen across retries");
          frozen = command.mode_request;
        }
        auto result = executor.update(command, s);
        check(!result.feedback.fault, "all six directed transitions must execute without fault");
        for (double speed : result.wheel_speed_targets)
          check(speed == 0, "transition and handover must never command drive");
        if (!result.feedback.confirmed)
          check(result.feedback.actual_mode == from,
                "requested mode must not replace actual mode early");
        if (!is_stopped(s, c))
          check(result.steering_targets == s.steering_angles, "moving wheels must retain steering");
        if (!manager.active()) {
          check(command.action == Action::Hold && s.actual_mode == to && frozen &&
                    s.mode_request_id == frozen->id,
                "matching acknowledgement must produce stopped handover");
          check(!DriveModel(c).step(s, intent(to), c.dt_s).aligning,
                "entry intent must avoid a second alignment in the same direction");
          finished = true;
          break;
        }
        actuate(s, result, c);
      }
      check(finished, "each directed transition must finish before its deadline");
    }
  }
}
void test_stale_ack_and_measured_alignment() {
  Config c;
  ModeManager m(c);
  VehicleState s;
  s.stamp_s = 1;
  s.mode_request_id = 20;
  m.begin(DriveMode::Crab, intent(DriveMode::Crab), s);
  auto out = m.update(s);
  check(out.mode_request && out.mode_request->id == 21,
        "request ID must follow observed high-water mark");
  s.actual_mode = DriveMode::Crab;
  s.mode_request_id = 20;
  s.steering_angles = out.steering_targets;
  s.stamp_s += .1;
  check(m.update(s).action == Action::RequestMode,
        "old acknowledgement must never complete a new request");
  s.mode_request_id = 21;
  s.steering_angles.fill(0);
  s.stamp_s += .1;
  check(m.update(s).action == Action::RequestMode,
        "acknowledgement without measured alignment must wait");
  s.steering_angles = out.steering_targets;
  s.wheel_speeds.fill(.01);
  s.stamp_s += .1;
  check(m.update(s).action == Action::RequestMode,
        "matching acknowledgement with moving wheels must wait");
  s.wheel_speeds.fill(0);
  s.stamp_s += .1;
  check(m.update(s).action == Action::Hold, "matching stopped aligned feedback must complete");
  m.reset();
  s.stamp_s += .1;
  m.begin(DriveMode::Spin, intent(DriveMode::Spin), s);
  check(m.update(s).mode_request->id == 22, "reset must not reuse request IDs");
}
void test_executor_idempotency_and_timeout() {
  Config c;
  c.alignment_min_s = 0;
  ModeExecutor executor(c);
  VehicleState s;
  s.stamp_s = 1;
  auto command = request(4, DriveMode::Crab, c);
  auto out = executor.update(command, s);
  check(!out.feedback.confirmed, "unaligned wheels must not confirm");
  s.steering_angles = command.steering_targets;
  s.stamp_s += .1;
  out = executor.update(command, s);
  check(out.feedback.confirmed && out.feedback.request_id == 4,
        "alignment must produce typed acknowledgement");
  s.stamp_s += .1;
  out = executor.update(command, s);
  check(out.feedback.confirmed && out.feedback.time_in_mode_s > .09,
        "completed retry must not restart mode age or transition");
  command.mode_request->steering_targets.fill(0);
  s.stamp_s += .1;
  check(executor.update(command, s).feedback.fault,
        "same ID with mutated payload must latch fault");

  ModeExecutor timeout(c);
  s = {};
  s.stamp_s = 1;
  command = request(1, DriveMode::Crab, c);
  check(!timeout.update(command, s).feedback.fault, "valid first request must be accepted");
  s.stamp_s = 1 + c.confirmation_timeout_s + .01;
  check(timeout.update(command, s).feedback.fault, "retry must not renew timeout");
  s.stamp_s += .1;
  s.steering_angles = command.steering_targets;
  check(timeout.update(command, s).feedback.fault, "late alignment must not clear a fault");
}
void test_cancellation_recovery_and_invalid_commands() {
  Config c;
  ModeExecutor e(c);
  VehicleState s;
  s.stamp_s = 1;
  auto command = request(8, DriveMode::Crab, c);
  e.update(command, s);
  Output stop;
  s.stamp_s += .1;
  check(e.update(stop, s).feedback.fault, "SafeStop must cancel and latch an active transition");
  s.mode_request_id = 8;
  e.reset(s);
  s.stamp_s += .1;
  check(e.update(command, s).feedback.fault, "delayed request before recovery must be rejected");
  e.reset(s);
  s.stamp_s += .1;
  command = request(9, DriveMode::Crab, c);
  check(!e.update(command, s).feedback.fault, "recovery must accept a fresh larger request ID");
  s.wheel_speeds.fill(.1);
  bool rejected = false;
  try {
    e.reset(s);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "recovery cannot be performed while wheels are moving");

  ModeExecutor invalid(c);
  s = {};
  s.stamp_s = 1;
  command = request(1, DriveMode::Spin, c);
  command.mode_request->steering_targets[0] = std::numeric_limits<double>::quiet_NaN();
  check(invalid.update(command, s).feedback.fault, "nonfinite request must safe stop");
  ModeExecutor missing(c);
  command.mode_request.reset();
  check(missing.update(command, s).feedback.fault, "RequestMode requires typed payload");
}
void test_request_geometry_and_id_exhaustion() {
  Config c;
  VehicleState s;
  s.stamp_s = 1;
  for (auto mode : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
    const auto initial =
        mode == DriveMode::DualAckermann ? DriveMode::Spin : DriveMode::DualAckermann;
    ModeExecutor executor(c, initial);
    auto command = request(1, mode, c);
    command.mode_request->steering_targets = {0, .3, 0, 0};
    check(executor.update(command, s).feedback.fault,
          "joint bounds alone must not admit incompatible mode-entry geometry");
  }
  s.mode_request_id = std::numeric_limits<std::uint64_t>::max();
  ModeManager manager(c);
  bool exhausted = false;
  try {
    manager.begin(DriveMode::Crab, intent(DriveMode::Crab), s);
  } catch (const std::overflow_error &) {
    exhausted = true;
  }
  check(exhausted && !manager.active(), "request IDs must never wrap to zero");
}
void test_persistent_mode_and_unconfirmed_drive() {
  Config c;
  ModeExecutor e(c, DriveMode::Crab);
  VehicleState s;
  s.actual_mode = DriveMode::Crab;
  s.stamp_s = 1;
  Output hold;
  hold.action = Action::Hold;
  hold.steering_targets = s.steering_angles;
  check(e.update(hold, s).feedback.actual_mode == DriveMode::Crab,
        "zero drive must preserve explicit mode");
  auto command = request(1, DriveMode::Spin, c);
  s.stamp_s += .1;
  e.update(command, s);
  Output drive;
  drive.action = Action::Drive;
  drive.requested_mode = DriveMode::Spin;
  drive.body_command.wz = .5;
  drive.wheel_speed_targets.fill(.4);
  s.stamp_s += .1;
  auto out = e.update(drive, s);
  check(!out.feedback.confirmed && out.feedback.actual_mode == DriveMode::Crab,
        "drive must not override an unconfirmed transition");
  for (double v : out.wheel_speed_targets)
    check(v == 0, "pending transition must mask all drive targets");
  s.stamp_s -= .1;
  check(e.update(hold, s).feedback.fault, "backward feedback clock must latch fault");
}
void test_boot_age_drive_consistency_and_preemption() {
  Config c;
  VehicleState s;
  s.stamp_s = 1;
  s.time_in_mode_s = 4;
  ModeExecutor age(c);
  Output hold;
  hold.action = Action::Hold;
  check(age.update(hold, s).feedback.time_in_mode_s == 4,
        "startup mode age may precede the nonnegative clock origin");
  s.stamp_s += .1;
  check(age.update(hold, s).feedback.time_in_mode_s > 4.09,
        "negative mode-entry time must not reinitialize age on each tick");

  ModeExecutor mismatch(c);
  Output drive;
  drive.action = Action::Drive;
  drive.body_command.vx = .2;
  drive.wheel_speed_targets.fill(.4);
  check(mismatch.update(drive, s).feedback.fault,
        "inconsistent body and wheel drive commands must fault");

  ModeExecutor preemption(c);
  auto command = request(1, DriveMode::Crab, c);
  preemption.update(command, s);
  s.stamp_s += .1;
  check(preemption.update(request(2, DriveMode::Spin, c), s).feedback.fault,
        "a fresh ID cannot silently replace an active transition");
}
void test_transition_prediction_deadline() {
  Config c;
  c.max_steer_rate_radps = .1;
  c.horizon_steps = 200;
  c.minimum_mode_dwell_s = 0;
  VehicleState s;
  std::size_t steps = 0;
  check(TransitionModel(c).rollout(s, DriveMode::Crab, steps, c.horizon_steps, nullptr,
                                   intent(DriveMode::Crab)) < 0,
        "prediction must reject alignment exceeding the real execution deadline");
}
void test_controller_executor_lateral_loop() {
  Config c;
  c.horizon_steps = 32;
  c.samples_per_branch = 24;
  c.minimum_mode_dwell_s = 0;
  c.switch_cost = .05;
  c.switch_hysteresis = .01;
  c.noise_v_mps = c.noise_w_radps = 0;
  Controller controller(c);
  ModeExecutor executor(c);
  ControllerInput input;
  input.vehicle.stamp_s = 1;
  input.vehicle.time_in_mode_s = 2;
  input.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
  bool requested = false, drove = false;
  for (int tick = 0; tick < 35; ++tick) {
    auto command = controller.compute(input);
    requested = requested || command.action == Action::RequestMode;
    if (command.action == Action::Drive) {
      check(input.vehicle.mode_confirmed && input.vehicle.actual_mode == DriveMode::Crab,
            "lateral drive requires acknowledged actual Crab mode");
      drove = true;
    }
    auto out = executor.update(command, input.vehicle);
    check(!out.feedback.fault, "controller/executor lateral closed loop must not fault");
    actuate(input.vehicle, out, c);
  }
  check(requested && drove && input.vehicle.pose.y > .7 && std::abs(input.vehicle.pose.x) < .02,
        "typed closed loop must make lateral progress after one explicit mode entry");
}
void test_measured_steering_tolerance() {
  Config c;
  VehicleState s;
  s.stamp_s = 1;
  const Control control{.4, 0, .2};
  s.steering_angles = Kinematics(c).inverse(control, {}).angles;
  for (double &angle : s.steering_angles)
    angle += .01;
  const auto predicted = DriveModel(c).step(s, control, c.dt_s);
  check(predicted.valid && !predicted.aligning && predicted.state.velocity.vy != 0,
        "small measured steering error must produce a consistent nonideal body twist");
  Output command;
  command.action = Action::Drive;
  command.body_command = predicted.state.velocity;
  command.steering_targets = predicted.steering_targets;
  command.wheel_speed_targets = predicted.wheel_speed_targets;
  ModeExecutor executor(c);
  check(!executor.update(command, s).feedback.fault,
        "bounded steering tracking error must not reject valid controller drive output");

  s = {};
  s.actual_mode = DriveMode::Crab;
  s.stamp_s = 1;
  // Consistent wheel/body translation remains incompatible with Spin geometry.
  ModeExecutor incompatible(c, DriveMode::Spin);
  command.requested_mode = DriveMode::Spin;
  command.steering_targets.fill(0);
  command.wheel_speed_targets.fill(.4);
  command.body_command = {.4, 0, 0};
  check(incompatible.update(command, s).feedback.fault,
        "tracking tolerance cannot admit a body twist from the wrong mode");
}
void test_nonfinite_drive_commands() {
  Config c;
  VehicleState s;
  s.stamp_s = 1;
  for (double bad :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    for (int axis = 0; axis < 3; ++axis) {
      ModeExecutor executor(c);
      Output command;
      command.action = Action::Drive;
      command.wheel_speed_targets.fill(.1);
      command.body_command = {.1, 0, 0};
      if (axis == 0)
        command.body_command.vx = bad;
      if (axis == 1)
        command.body_command.vy = bad;
      if (axis == 2)
        command.body_command.wz = bad;
      const auto result = executor.update(command, s);
      check(result.feedback.fault && result.action == Action::SafeStop,
            "every nonfinite body command component must latch a fault");
      for (double speed : result.wheel_speed_targets)
        check(speed == 0, "invalid drive must never reach wheel targets");
    }
  }
}
void test_frozen_controller_alignment() {
  Config c;
  c.minimum_mode_dwell_s = 100;
  c.noise_v_mps = c.noise_w_radps = 0;
  auto initial = [] {
    ControllerInput in;
    in.vehicle.actual_mode = DriveMode::Crab;
    in.vehicle.stamp_s = 1;
    in.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
    return in;
  };
  Controller controller(c);
  auto in = initial();
  const auto first = controller.compute(in);
  check(first.action == Action::Hold, "large same-mode steering must start stopped alignment");
  // Repeated updates of the same path cannot chase a committed steering target.
  in.vehicle.stamp_s += c.dt_s;
  const auto retry = controller.compute(in);
  check(retry.action == Action::Hold && retry.steering_targets == first.steering_targets,
        "same-mode alignment must keep its entry intent across solves");
  in.vehicle.stamp_s += c.confirmation_timeout_s;
  check(controller.compute(in).action == Action::SafeStop,
        "a stalled steering actuator must time out instead of holding forever");
  controller.reset();
  in = initial();
  check(controller.compute(in).action == Action::Hold, "reset must clear stale alignment");
  in.vehicle.stamp_s += c.dt_s;
  in.obstacles = {{0, 0, .1}};
  check(controller.compute(in).action == Action::SafeStop,
        "new obstacles must be checked during committed local alignment");
}
void test_drive_steering_command_limits() {
  Config c;
  c.max_steer_rate_radps = .1;
  for (double delta : {.02, .3}) {
    VehicleState s;
    s.actual_mode = DriveMode::Crab;
    s.stamp_s = 1;
    Output command;
    command.action = Action::Drive;
    command.requested_mode = DriveMode::Crab;
    command.steering_targets.fill(delta);
    command.wheel_speed_targets.fill(.2);
    command.body_command = {.2 * std::cos(delta), .2 * std::sin(delta), 0};
    ModeExecutor executor(c, DriveMode::Crab);
    check(executor.update(command, s).feedback.fault,
          "executor must reject steering jumps beyond rate or moving-angle limits");
  }
}
void test_module_velocity_residuals() {
  Config c;
  VehicleState state;
  state.stamp_s = 1;
  for (auto speeds :
       {std::array<double, 4>{.5, -.5, -.5, .5}, std::array<double, 4>{.5, -.5, .5, -.5}}) {
    Output command;
    command.action = Action::Drive;
    command.wheel_speed_targets = speeds;
    command.body_command = Kinematics(c).forward(speeds, command.steering_targets);
    const auto result = ModeExecutor(c).update(command, state);
    check(result.action == Action::SafeStop && result.feedback.fault &&
              result.wheel_speed_targets == std::array<double, 4>{},
          "least-squares body agreement cannot hide incompatible module velocity vectors");
  }
  for (double residual : {.01, .03}) {
    Output command;
    command.action = Action::Drive;
    command.wheel_speed_targets = {.2 + residual, .2 - residual, .2 - residual, .2 + residual};
    command.body_command = {.2, 0, 0};
    check(ModeExecutor(c).update(command, state).feedback.fault == (residual > .02),
          "per-module residual allowance must be explicit and bounded");
  }
  c.drive_kinematic_tolerance_mps = 0;
  Output exact;
  exact.action = Action::Drive;
  exact.wheel_speed_targets.fill(.2);
  exact.body_command = {.2, 0, 0};
  check(!ModeExecutor(c).update(exact, state).feedback.fault,
        "zero module tolerance must permit ideal rigid-body commands");
}
void test_curved_ackermann_and_spin_drive() {
  Config c;
  c.minimum_mode_dwell_s = 100;
  c.samples_per_branch = 8;
  c.noise_v_mps = c.noise_w_radps = 0;
  for (auto mode : {DriveMode::DualAckermann, DriveMode::Spin}) {
    Controller controller(c);
    ModeExecutor executor(c, mode);
    ControllerInput input;
    input.vehicle.actual_mode = mode;
    input.vehicle.stamp_s = 1;
    input.reference_path = mode == DriveMode::Spin ? std::vector<Pose2d>{{0, 0, 0}, {0, 0, .8}}
                                                   : std::vector<Pose2d>{{0, 0, 0}, {1, .4, .4}};
    input.vehicle.steering_angles = DriveModel(c).steering_for_mode(mode);
    bool drove = false;
    for (int tick = 0; tick < 18; ++tick) {
      auto command = controller.compute(input);
      drove = drove || command.action == Action::Drive;
      auto execution = executor.update(command, input.vehicle);
      check(!execution.feedback.fault, "valid curved Ackermann and Spin commands must execute "
                                       "despite steering tracking tolerance");
      actuate(input.vehicle, execution, c);
    }
    check(drove &&
              (mode == DriveMode::Spin ? input.vehicle.pose.yaw > .3 : input.vehicle.pose.x > .2),
          "stable mode execution must make curved/rotational progress");
  }
}
void test_rollout_entry_direction() {
  Config c;
  c.horizon_steps = 32;
  c.minimum_mode_dwell_s = 0;
  VehicleState s;
  for (std::size_t switch_step : {0u, 3u}) {
    std::vector<Control> controls(c.horizon_steps, {.4, 0, 0});
    controls[switch_step] = intent(DriveMode::Crab);
    auto trajectory = RolloutEngine(c).generate(s, {DriveMode::Crab, switch_step, true}, controls);
    check(trajectory.valid, "direction-aware switch rollout must be feasible");
    auto confirmed = s;
    for (std::size_t i = 0; i < switch_step; ++i)
      confirmed = DriveModel(c).step(confirmed, controls[i], c.dt_s).state;
    auto resume = switch_step;
    check(TransitionModel(c).rollout(confirmed, DriveMode::Crab, resume, c.horizon_steps, nullptr,
                                     controls[switch_step]) >= 0,
          "entry transition must fit the horizon");
    const auto drive = DriveModel(c).step(confirmed, controls[switch_step], c.dt_s);
    check(!drive.aligning && !trajectory.active_controls[resume] &&
              trajectory.controls[resume].vx == 0 && trajectory.controls[resume].vy > 0 &&
              std::abs(trajectory.poses[resume + 1].x - drive.state.pose.x) < 1e-9 &&
              std::abs(trajectory.poses[resume + 1].y - drive.state.pose.y) < 1e-9,
          "first Drive must use frozen entry intent, ignoring a different resume proposal");
  }
}
void test_capture_alignment_commitment() {
  Config c;
  for (bool corner : {false, true}) {
    Controller controller(c);
    ControllerInput in;
    in.vehicle.actual_mode = DriveMode::Crab;
    in.vehicle.time_in_mode_s = 2;
    in.vehicle.stamp_s = 1;
    in.reference_path = corner ? std::vector<Pose2d>{{0, 0, 0}, {0, .15, 0}, {.5, .15, 0}}
                               : std::vector<Pose2d>{{0, 0, 0}, {0, .1, 0}};
    const auto first = controller.compute(in);
    check(first.control_policy == ControlPolicy::Capture && first.action == Action::Hold,
          "terminal/corner capture must begin stopped same-mode alignment");
    in.vehicle.pose.x = .01;
    in.vehicle.stamp_s += c.dt_s;
    const auto retry = controller.compute(in);
    check(retry.control_policy == ControlPolicy::Alignment && retry.action == Action::Hold &&
              retry.steering_targets == first.steering_targets,
          "capture must retain its steering intent despite measured pose jitter");
    in.vehicle.stamp_s = 1 + c.confirmation_timeout_s + c.dt_s;
    const auto timeout = controller.compute(in);
    check(timeout.action == Action::SafeStop &&
              timeout.failure_reason == FailureReason::TransitionFault,
          "capture alignment must honor the same execution deadline as tracking");
  }
  Controller controller(c);
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 0}, {0, .1, 0}};
  const auto request = controller.compute(in);
  check(request.mode_request && request.requested_mode == DriveMode::Crab,
        "terminal lateral capture must request Crab entry");
  in.vehicle.actual_mode = DriveMode::Crab;
  in.vehicle.mode_request_id = request.mode_request->id;
  in.vehicle.steering_angles = request.steering_targets;
  in.vehicle.stamp_s += c.dt_s;
  check(controller.compute(in).action == Action::Hold, "confirmation must retain stopped handover");
  in.vehicle.pose.x = .01;
  in.vehicle.stamp_s += c.dt_s;
  const auto drive = controller.compute(in);
  check(drive.action == Action::Drive && drive.body_command.vy > 0 &&
            std::abs(drive.body_command.vx) < 1e-9,
        "capture cannot overwrite agreed mode-entry intent before its first Drive");
}
} // namespace
int main() {
  try {
    test_all_directed_transitions();
    test_stale_ack_and_measured_alignment();
    test_executor_idempotency_and_timeout();
    test_cancellation_recovery_and_invalid_commands();
    test_request_geometry_and_id_exhaustion();
    test_persistent_mode_and_unconfirmed_drive();
    test_boot_age_drive_consistency_and_preemption();
    test_transition_prediction_deadline();
    test_controller_executor_lateral_loop();
    test_measured_steering_tolerance();
    test_nonfinite_drive_commands();
    test_frozen_controller_alignment();
    test_drive_steering_command_limits();
    test_module_velocity_residuals();
    test_curved_ackermann_and_spin_drive();
    test_rollout_entry_direction();
    test_capture_alignment_commitment();
    std::cout << "Execution regressions passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
