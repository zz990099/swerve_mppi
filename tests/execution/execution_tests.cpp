#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "behavior_fixture.hpp"
#include "planning/detail/planner.hpp"
#include "swerve_mppi/execution/executor.hpp"

using namespace swerve_mppi;
namespace
{
void check(bool ok, const char * message)
{
  if (!ok) {
    throw std::runtime_error(message);
  }
}
Control intent(DriveMode mode)
{
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
void feedback(VehicleState & s, const ModeFeedback & f)
{
  s.actual_mode = f.actual_mode;
  s.mode_confirmed = f.confirmed;
  s.mode_fault = f.fault;
  s.mode_request_id = f.request_id;
  s.accepted_mode_request = f.accepted_mode_request;
  s.time_in_mode_s = f.time_in_mode_s;
}
void actuate(VehicleState & s, const ExecutionResult & r, const Config & c)
{
  test::actuate(s, r, c);
}

JointCommand request(std::uint64_t id, DriveMode mode, const Config & c)
{
  JointCommand out;
  out.action = Action::RequestMode;
  out.requested_mode = mode;
  out.mode_request =
    JointModeRequest{id, mode, DriveModel(c).steering_for_entry(mode, intent(mode), {})};
  out.steering_targets = out.mode_request->steering_targets;
  return out;
}
void test_all_directed_transitions()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  for (auto from : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
    for (auto to : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
      if (from == to) {
        continue;
      }
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
      std::optional<JointModeRequest> frozen;
      bool finished = false;
      for (int tick = 0; tick < 40; ++tick) {
        auto command = manager.update(s);
        if (command.mode_request) {
          if (frozen) {
            check(
              command.mode_request->id == frozen->id &&
                command.mode_request->steering_targets == frozen->steering_targets,
              "request payload must remain frozen across retries");
          }
          frozen = command.mode_request;
        }
        auto result = executor.update(command, s);
        check(!result.feedback.fault, "all six directed transitions must execute without fault");
        for (double speed : result.wheel_speed_targets) {
          check(speed == 0, "transition and handover must never command drive");
        }
        if (!result.feedback.confirmed) {
          check(
            result.feedback.actual_mode == from,
            "requested mode must not replace actual mode early");
        }
        if (!is_stopped(s, c)) {
          check(result.steering_targets == s.steering_angles, "moving wheels must retain steering");
        }
        if (!manager.active()) {
          check(
            command.action == Action::Hold && s.actual_mode == to && frozen &&
              s.mode_request_id == frozen->id,
            "matching acknowledgement must produce stopped handover");
          check(
            !DriveModel(c).step(s, intent(to), c.dt_s).aligning,
            "entry intent must avoid a second alignment in the same "
            "direction");
          finished = true;
          break;
        }
        actuate(s, result, c);
      }
      check(finished, "each directed transition must finish before its deadline");
    }
  }
}
void test_stale_ack_and_measured_alignment()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  ModeManager m(c);
  VehicleState s;
  s.stamp_s = 1;
  s.mode_request_id = 20;
  m.begin(DriveMode::Crab, intent(DriveMode::Crab), s);
  auto out = m.update(s);
  check(
    out.mode_request && out.mode_request->id == 21,
    "request ID must follow observed high-water mark");
  s.actual_mode = DriveMode::Crab;
  s.mode_request_id = 20;
  s.steering_angles = out.steering_targets;
  s.stamp_s += .1;
  check(
    m.update(s).action == Action::RequestMode,
    "old acknowledgement must never complete a new request");
  s.mode_request_id = 21;
  s.accepted_mode_request = out.mode_request;
  s.steering_angles.fill(0);
  s.stamp_s += .1;
  check(
    m.update(s).action == Action::RequestMode,
    "acknowledgement without measured alignment must wait");
  s.steering_angles = out.steering_targets;
  s.wheel_speeds.fill(.01);
  s.velocity = Kinematics(c).forward(s.wheel_speeds, s.steering_angles);
  s.stamp_s += .1;
  check(
    m.update(s).action == Action::RequestMode,
    "matching acknowledgement with moving wheels must wait");
  s.wheel_speeds.fill(0);
  s.velocity = {};
  s.stamp_s += .1;
  check(m.update(s).action == Action::Hold, "matching stopped aligned feedback must complete");
  m.reset();
  s.stamp_s += .1;
  m.begin(DriveMode::Spin, intent(DriveMode::Spin), s);
  check(m.update(s).mode_request->id == 22, "reset must not reuse request IDs");
}
void test_executor_idempotency_and_timeout()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
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
  check(
    out.feedback.confirmed && out.feedback.request_id == 4,
    "alignment must produce typed acknowledgement");
  s.stamp_s += .1;
  out = executor.update(command, s);
  check(
    out.feedback.confirmed && out.feedback.time_in_mode_s > .09,
    "completed retry must not restart mode age or transition");
  command.mode_request->steering_targets.fill(0);
  s.stamp_s += .1;
  check(
    executor.update(command, s).feedback.fault, "same ID with mutated payload must latch fault");

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
void test_cancellation_recovery_and_invalid_commands()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  ModeExecutor e(c);
  VehicleState s;
  s.stamp_s = 1;
  auto command = request(8, DriveMode::Crab, c);
  e.update(command, s);
  JointCommand stop;
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
void test_request_geometry_and_id_exhaustion()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  VehicleState s;
  s.stamp_s = 1;
  for (auto mode : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
    const auto initial =
      mode == DriveMode::DualAckermann ? DriveMode::Spin : DriveMode::DualAckermann;
    ModeExecutor executor(c, initial);
    auto command = request(1, mode, c);
    command.mode_request->steering_targets = {0, .3, 0, 0};
    check(
      executor.update(command, s).feedback.fault,
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
void test_persistent_mode_and_unconfirmed_drive()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  ModeExecutor e(c, DriveMode::Crab);
  VehicleState s;
  s.actual_mode = DriveMode::Crab;
  s.stamp_s = 1;
  JointCommand hold;
  hold.action = Action::Hold;
  hold.steering_targets = s.steering_angles;
  check(
    e.update(hold, s).feedback.actual_mode == DriveMode::Crab,
    "zero drive must preserve explicit mode");
  auto command = request(1, DriveMode::Spin, c);
  s.stamp_s += .1;
  e.update(command, s);
  JointCommand drive;
  drive.action = Action::Drive;
  drive.requested_mode = DriveMode::Spin;
  drive.body_command.wz = .5;
  drive.wheel_speed_targets.fill(.4);
  s.stamp_s += .1;
  auto out = e.update(drive, s);
  check(
    !out.feedback.confirmed && out.feedback.actual_mode == DriveMode::Crab,
    "drive must not override an unconfirmed transition");
  for (double v : out.wheel_speed_targets) {
    check(v == 0, "pending transition must mask all drive targets");
  }
  s.stamp_s -= .1;
  check(e.update(hold, s).feedback.fault, "backward feedback clock must latch fault");
}
void test_boot_age_drive_consistency_and_preemption()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  VehicleState s;
  s.stamp_s = 1;
  s.time_in_mode_s = 4;
  ModeExecutor age(c);
  JointCommand hold;
  hold.action = Action::Hold;
  check(
    age.update(hold, s).feedback.time_in_mode_s == 4,
    "startup mode age may precede the nonnegative clock origin");
  s.stamp_s += .1;
  check(
    age.update(hold, s).feedback.time_in_mode_s > 4.09,
    "negative mode-entry time must not reinitialize age on each tick");

  ModeExecutor mismatch(c);
  JointCommand drive;
  drive.action = Action::Drive;
  drive.body_command.vx = .2;
  drive.wheel_speed_targets.fill(.4);
  check(
    mismatch.update(drive, s).feedback.fault,
    "inconsistent body and wheel drive commands must fault");

  ModeExecutor preemption(c);
  auto command = request(1, DriveMode::Crab, c);
  preemption.update(command, s);
  s.stamp_s += .1;
  check(
    preemption.update(request(2, DriveMode::Spin, c), s).feedback.fault,
    "a fresh ID cannot silently replace an active transition");
}
void test_transition_prediction_deadline()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.max_steer_rate_radps = .1;
  c.horizon_steps = 200;
  c.minimum_mode_dwell_s = 0;
  VehicleState s;
  std::size_t steps = 0;
  check(
    TransitionModel(c).rollout(
      s, DriveMode::Crab, steps, c.horizon_steps, nullptr, intent(DriveMode::Crab)) < 0,
    "prediction must reject alignment exceeding the real execution deadline");
}
void test_transition_protocol_tick_matrix()
{
  // Compare predicted first motion with a protocol loop advanced by the
  // independent encoder fixture, including deliberately delayed confirmation.
  for (double dt : {.05, .1, .2}) {
    for (double allowance : {0.0, .05, .1, .2, .35}) {
      for (double alignment : {0.0, .13}) {
        for (double steer_rate : {2.5, 10.0}) {
          Config c;
          c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
          c.dt_s = dt;
          c.confirmation_prediction_s = allowance;
          c.alignment_min_s = alignment;
          c.max_steer_rate_radps = steer_rate;
          c.minimum_mode_dwell_s = 0;
          c.horizon_steps = 80;
          for (auto from : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
            for (auto to : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
              if (from == to) {
                continue;
              }
              for (std::size_t switch_step : {0u, 3u}) {
                VehicleState state;
                state.actual_mode = from;
                state.stamp_s = 1;
                state.time_in_mode_s = 2;
                state.steering_angles = DriveModel(c).steering_for_entry(from, intent(from), {});
                std::vector<Control> controls(c.horizon_steps);
                controls[switch_step] = intent(to);
                const auto trace =
                  RolloutEngine(c).generate(state, {to, switch_step, true}, controls);
                check(trace.valid, "parameterized transition prediction must fit its horizon");
                std::size_t predicted = c.horizon_steps;
                for (std::size_t step = 0; step + 1 < trace.poses.size(); ++step) {
                  const auto &a = trace.poses[step], &b = trace.poses[step + 1];
                  if (
                    std::hypot(b.x - a.x, b.y - a.y) > 1e-9 ||
                    std::abs(angle_distance(b.yaw, a.yaw)) > 1e-9) {
                    predicted = step;
                    break;
                  }
                }
                ModeManager manager(c);
                ModeExecutor executor(c, from);
                const std::size_t allowance_ticks =
                  static_cast<std::size_t>(std::ceil(allowance / dt));
                const std::size_t delivery_delay = allowance_ticks > 2 ? allowance_ticks - 2 : 0;
                std::optional<std::size_t> confirmed_tick;
                std::size_t actual = c.horizon_steps;
                for (std::size_t tick = 0; tick < c.horizon_steps; ++tick) {
                  JointCommand command;
                  command.action = Action::Hold;
                  command.requested_mode = state.actual_mode;
                  command.steering_targets = state.steering_angles;
                  if (tick == switch_step) {
                    manager.begin(to, intent(to), state);
                  }
                  if (tick >= switch_step) {
                    if (manager.active()) {
                      command = manager.update(state);
                    } else {
                      actual = tick;
                      break;
                    }
                  }
                  auto result = executor.update(command, state);
                  check(
                    !result.feedback.fault && result.action != Action::Drive &&
                      command.action != Action::SafeStop,
                    "prediction comparison must retain the measured "
                    "confirmation gate");
                  if (result.feedback.confirmed && result.feedback.actual_mode == to) {
                    if (!confirmed_tick) {
                      confirmed_tick = tick;
                    }
                    if (tick - *confirmed_tick < delivery_delay) {
                      result.feedback.actual_mode = from;
                      result.feedback.confirmed = false;
                      // Delay completion, retaining the already accepted request receipt.
                    }
                  }
                  test::actuate(state, result, c);
                }
                check(
                  predicted == actual && actual < c.horizon_steps,
                  "predicted first motion must match protocol confirmation "
                  "and handover ticks");
              }
            }
          }
        }
      }
    }
  }
}
void test_confirmation_receipt_deadline()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.dt_s = .125;
  c.alignment_min_s = c.confirmation_prediction_s = 0;
  c.confirmation_timeout_s = c.dt_s;
  VehicleState initial;
  initial.stamp_s = 1;
  std::size_t steps = 0;
  auto predicted = initial;
  check(
    TransitionModel(c).rollout(predicted, DriveMode::Crab, steps, 4, nullptr, {.2, 0, 0}) == .25 &&
      steps == 2,
    "confirmation receipt may meet its deadline before the handover cycle "
    "ends");
  ModeManager manager(c);
  ModeExecutor executor(c);
  manager.begin(DriveMode::Crab, {.2, 0, 0}, initial);
  auto state = initial;
  for (int tick = 0; tick < 2; ++tick) {
    const auto command = manager.update(state);
    const auto result = executor.update(command, state);
    check(
      command.action != Action::SafeStop && !result.feedback.fault,
      "matching confirmation at the receipt deadline must remain executable");
    test::actuate(state, result, c);
  }
  check(!manager.active(), "deadline receipt must complete measured handover");
  steps = 0;
  predicted = initial;
  check(
    TransitionModel(c).rollout(predicted, DriveMode::Crab, steps, 1, nullptr, {.2, 0, 0}) < 0,
    "one remaining horizon tick cannot contain both mandatory stopped "
    "cycles");
  c.confirmation_timeout_s = .12;
  steps = 0;
  predicted = initial;
  check(
    TransitionModel(c).rollout(predicted, DriveMode::Crab, steps, 4, nullptr, {.2, 0, 0}) < 0,
    "prediction must reject receipt after the execution deadline");
}
void test_decimal_deadline_and_tick_boundaries()
{
  for (double dt : {.01, .04, .1, .125}) {
    for (double stamp : {1.0, 1e6, 1700000000.0}) {
      for (double lateness : {0.0, 1e-5}) {
        Config c;
        c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
        c.dt_s = dt;
        c.alignment_min_s = c.confirmation_prediction_s = 0;
        c.confirmation_timeout_s = dt;
        VehicleState state;
        state.stamp_s = stamp;
        ModeManager manager(c);
        ModeExecutor executor(c);
        manager.begin(DriveMode::Crab, {.2, 0, 0}, state);
        const auto command = manager.update(state);
        const auto confirmed = executor.update(command, state);
        check(confirmed.feedback.confirmed, "decimal boundary setup must confirm immediately");
        state.actual_mode = confirmed.feedback.actual_mode;
        state.mode_confirmed = true;
        state.mode_request_id = confirmed.feedback.request_id;
        state.accepted_mode_request = confirmed.feedback.accepted_mode_request;
        state.stamp_s = stamp + dt + lateness;
        check(
          (manager.update(state).action == Action::SafeStop) == (lateness > 0),
          "decimal confirmation receipt must accept the deadline and "
          "reject actual lateness");

        // Exercise the executor's own inclusive deadline while still aligning.
        state = {};
        state.stamp_s = stamp;
        c.alignment_min_s = dt;
        ModeManager requestor(c);
        ModeExecutor aligning(c);
        requestor.begin(DriveMode::Crab, {.2, 0, 0}, state);
        const auto request = requestor.update(state);
        check(
          !aligning.update(request, state).feedback.confirmed,
          "executor deadline setup must retain an active request");
        state.stamp_s = stamp + dt + lateness;
        check(
          aligning.update(request, state).feedback.fault == (lateness > 0),
          "executor and manager must agree on inclusive decimal deadlines");
      }
    }
  }
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.dt_s = .04;
  c.alignment_min_s = .28;  // .28/.04 rounds above seven in binary floating point.
  c.confirmation_prediction_s = 0;
  VehicleState state;
  state.stamp_s = 1;
  std::size_t steps = 0;
  auto predicted = state;
  check(
    TransitionModel(c).rollout(predicted, DriveMode::Crab, steps, 9, nullptr, {.2, 0, 0}) > 0 &&
      steps == 9,
    "decimal alignment quantization must reserve seven alignment and two "
    "protocol ticks");
  ModeManager manager(c);
  ModeExecutor executor(c);
  manager.begin(DriveMode::Crab, {.2, 0, 0}, state);
  for (int tick = 0; tick < 9; ++tick) {
    const auto command = manager.update(state);
    const auto result = executor.update(command, state);
    check(
      command.action != Action::SafeStop && !result.feedback.fault,
      "decimal quantization must execute without timeout");
    test::actuate(state, result, c);
    check(
      manager.active() == (tick < 8),
      "predicted decimal alignment and measured handover must finish on "
      "the same tick");
  }
  c.alignment_min_s = 0;
  c.confirmation_prediction_s = .28;
  steps = 0;
  predicted = {};
  check(
    TransitionModel(c).rollout(predicted, DriveMode::Crab, steps, 7) > 0 && steps == 7,
    "decimal confirmation allowances cannot gain a spurious prediction tick");
  c.horizon_steps = 20;
  c.minimum_mode_dwell_s = .28;
  const auto branches = ModeScheduler(c).make_branches({});
  check(
    branches.size() > 1 && branches[1].switch_step == 7,
    "the scheduler must use the same duration-to-ticks rule as transition "
    "prediction");
}
void test_same_mode_alignment_decimal_deadline()
{
  for (double lateness : {0.0, 1e-5}) {
    Config c;
    c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
    c.confirmation_timeout_s = c.dt_s;
    c.max_steer_rate_radps = 20;
    c.horizon_steps = 2;
    ControllerInput input;
    input.vehicle.actual_mode = DriveMode::Crab;
    input.vehicle.stamp_s = 1;
    input.reference_path = {{0, 0, 0}, {0, .1, 0}};
    std::vector<Control> controls(c.horizon_steps, {0, .12, 0});
    check(
      RolloutEngine(c).generate(input.vehicle, {}, controls).valid,
      "same-mode prediction must admit its first Drive at the alignment "
      "deadline");
    detail::Planner controller(c);
    ModeExecutor executor(c, DriveMode::Crab);
    const auto alignment = controller.compute(input);
    check(
      alignment.action == Action::Hold,
      "same-mode deadline setup must commit a stopped steering alignment");
    test::actuate(input.vehicle, executor.update(alignment, input.vehicle), c);
    input.vehicle.stamp_s += lateness;
    const auto command = controller.compute(input);
    check(
      command.action == (lateness > 0 ? Action::SafeStop : Action::Drive),
      "same-mode execution must accept the decimal boundary and reject "
      "actual lateness");
  }
}
void test_zero_delay_controller_capture_timing()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.alignment_min_s = c.confirmation_prediction_s = 0;
  c.max_steer_rate_radps = 10;
  ControllerInput input;
  input.vehicle.stamp_s = 1;
  input.vehicle.time_in_mode_s = 2;
  input.reference_path = {{0, 0, 0}, {.1, .1, 0}};
  std::vector<Control> controls(c.horizon_steps);
  controls[0] = {.12, .12, 0};
  const auto trace = RolloutEngine(c).generate(input.vehicle, {DriveMode::Crab, 0, true}, controls);
  check(
    trace.valid && std::hypot(trace.poses[3].x, trace.poses[3].y) < 1e-9 &&
      std::hypot(trace.poses[4].x, trace.poses[4].y) > 1e-9,
    "zero-delay diagonal entry must reserve three stopped ticks before "
    "first motion");
  detail::Planner controller(c);
  ModeExecutor executor(c);
  for (int tick = 0; tick <= 3; ++tick) {
    const auto command = controller.compute(input);
    check(
      (command.action == Action::Drive) == (tick == 3),
      "controller capture must share the predicted first-Drive timing");
    const auto result = executor.update(command, input.vehicle);
    check(!result.feedback.fault, "zero-delay capture must retain a healthy measured handshake");
    test::actuate(input.vehicle, result, c);
  }
}
void test_controller_executor_lateral_loop()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.horizon_steps = 32;
  c.samples_per_branch = 24;
  c.minimum_mode_dwell_s = 0;
  c.switch_cost = .05;
  c.switch_hysteresis = .01;
  c.noise_v_mps = c.noise_w_radps = 0;
  detail::Planner controller(c);
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
      check(
        input.vehicle.mode_confirmed && input.vehicle.actual_mode == DriveMode::Crab,
        "lateral drive requires acknowledged actual Crab mode");
      drove = true;
    }
    auto out = executor.update(command, input.vehicle);
    check(!out.feedback.fault, "controller/executor lateral closed loop must not fault");
    actuate(input.vehicle, out, c);
  }
  check(
    requested && drove && input.vehicle.pose.y > .7 && std::abs(input.vehicle.pose.x) < .02,
    "typed closed loop must make lateral progress after one explicit mode "
    "entry");
}
void test_measured_steering_tolerance()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  VehicleState s;
  s.stamp_s = 1;
  const Control control{.4, 0, .2};
  s.steering_angles = Kinematics(c).inverse(control, {}).angles;
  for (double & angle : s.steering_angles) {
    angle += .01;
  }
  const auto predicted = DriveModel(c).step(s, control, c.dt_s);
  check(
    predicted.valid && !predicted.aligning && predicted.state.velocity.vy != 0,
    "small measured steering error must produce a consistent nonideal body "
    "twist");
  JointCommand command;
  command.action = Action::Drive;
  command.body_command = predicted.state.velocity;
  command.steering_targets = predicted.steering_targets;
  command.wheel_speed_targets = predicted.wheel_speed_targets;
  ModeExecutor executor(c);
  check(
    !executor.update(command, s).feedback.fault,
    "bounded steering tracking error must not reject valid controller "
    "drive output");

  s = {};
  s.actual_mode = DriveMode::Crab;
  s.stamp_s = 1;
  // Consistent wheel/body translation remains incompatible with Spin geometry.
  ModeExecutor incompatible(c, DriveMode::Spin);
  command.requested_mode = DriveMode::Spin;
  command.steering_targets.fill(0);
  command.wheel_speed_targets.fill(.4);
  command.body_command = {.4, 0, 0};
  check(
    incompatible.update(command, s).feedback.fault,
    "tracking tolerance cannot admit a body twist from the wrong mode");
}
void test_nonfinite_drive_commands()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  VehicleState s;
  s.stamp_s = 1;
  for (double bad :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    for (int axis = 0; axis < 3; ++axis) {
      ModeExecutor executor(c);
      JointCommand command;
      command.action = Action::Drive;
      command.wheel_speed_targets.fill(.1);
      command.body_command = {.1, 0, 0};
      if (axis == 0) {
        command.body_command.vx = bad;
      }
      if (axis == 1) {
        command.body_command.vy = bad;
      }
      if (axis == 2) {
        command.body_command.wz = bad;
      }
      const auto result = executor.update(command, s);
      check(
        result.feedback.fault && result.action == Action::SafeStop,
        "every nonfinite body command component must latch a fault");
      for (double speed : result.wheel_speed_targets) {
        check(speed == 0, "invalid drive must never reach wheel targets");
      }
    }
  }
}
void test_frozen_controller_alignment()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.minimum_mode_dwell_s = 100;
  c.noise_v_mps = c.noise_w_radps = 0;
  auto initial = [] {
    ControllerInput in;
    in.vehicle.actual_mode = DriveMode::Crab;
    in.vehicle.stamp_s = 1;
    in.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
    return in;
  };
  detail::Planner controller(c);
  auto in = initial();
  const auto first = controller.compute(in);
  check(first.action == Action::Hold, "large same-mode steering must start stopped alignment");
  // Repeated updates of the same path cannot chase a committed steering target.
  in.vehicle.stamp_s += c.dt_s;
  const auto retry = controller.compute(in);
  check(
    retry.action == Action::Hold && retry.steering_targets == first.steering_targets,
    "same-mode alignment must keep its entry intent across solves");
  in.vehicle.stamp_s += c.confirmation_timeout_s;
  check(
    controller.compute(in).action == Action::SafeStop,
    "a stalled steering actuator must time out instead of holding forever");
  controller.reset();
  in = initial();
  check(controller.compute(in).action == Action::Hold, "reset must clear stale alignment");
  in.vehicle.stamp_s += c.dt_s;
  in.obstacles = {{0, 0, .1}};
  check(
    controller.compute(in).action == Action::SafeStop,
    "new obstacles must be checked during committed local alignment");
}
void test_drive_steering_command_limits()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.max_steer_rate_radps = .1;
  for (double delta : {.02, .3}) {
    VehicleState s;
    s.actual_mode = DriveMode::Crab;
    s.stamp_s = 1;
    JointCommand command;
    command.action = Action::Drive;
    command.requested_mode = DriveMode::Crab;
    command.steering_targets.fill(delta);
    command.wheel_speed_targets.fill(.2);
    command.body_command = {.2 * std::cos(delta), .2 * std::sin(delta), 0};
    ModeExecutor executor(c, DriveMode::Crab);
    check(
      executor.update(command, s).feedback.fault,
      "executor must reject steering jumps beyond rate or moving-angle "
      "limits");
  }
}
void test_absolute_speed_and_drive_interpolation_limits()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  for (auto mode : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
    const double maximum = mode == DriveMode::Crab   ? c.max_crab_speed_mps
                           : mode == DriveMode::Spin ? c.max_spin_radps
                                                     : c.max_vx_mps;
    for (double excess : {0.0, .01, .2}) {
      VehicleState state;
      state.stamp_s = 1;
      state.actual_mode = mode;
      const Control initial =
        mode == DriveMode::Spin ? Control{0, 0, maximum - .01} : Control{maximum - .01, 0, 0};
      const auto before = Kinematics(c).inverse(initial, {});
      state.steering_angles = before.angles;
      state.wheel_speeds = before.speeds;
      state.velocity = {initial.vx, initial.vy, initial.wz};
      const Control intent =
        mode == DriveMode::Spin ? Control{0, 0, maximum + excess} : Control{maximum + excess, 0, 0};
      const auto joint = Kinematics(c).inverse(intent, state.steering_angles);
      JointCommand command;
      command.action = Action::Drive;
      command.requested_mode = mode;
      command.steering_targets = joint.angles;
      command.wheel_speed_targets = joint.speeds;
      command.body_command = {intent.vx, intent.vy, intent.wz};
      check(
        ModeExecutor(c, mode).update(command, state).feedback.fault == (excess > 0),
        "absolute body speed limits cannot borrow steering interpolation "
        "tolerance");
    }
  }
  c.max_linear_decel_mps2 = .1;
  VehicleState state;
  state.stamp_s = 1;
  state.velocity.vx = .03;
  state.wheel_speeds.fill(.03);
  JointCommand command;
  command.action = Action::Drive;
  command.body_command.vx = -.04;
  command.wheel_speed_targets.fill(-.04);
  check(
    ModeExecutor(c).update(command, state).feedback.fault,
    "executor must reject a reversal whose affine ramp violates pointwise "
    "deceleration");
}
void test_module_velocity_residuals()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  VehicleState state;
  state.stamp_s = 1;
  for (auto speeds :
       {std::array<double, 4>{.5, -.5, -.5, .5}, std::array<double, 4>{.5, -.5, .5, -.5}}) {
    JointCommand command;
    command.action = Action::Drive;
    command.wheel_speed_targets = speeds;
    command.body_command = Kinematics(c).forward(speeds, command.steering_targets);
    const auto result = ModeExecutor(c).update(command, state);
    check(
      result.action == Action::SafeStop && result.feedback.fault &&
        result.wheel_speed_targets == std::array<double, 4>{},
      "least-squares body agreement cannot hide incompatible module "
      "velocity vectors");
  }
  state.wheel_speeds.fill(.2);
  state.velocity.vx = .2;
  for (double residual : {.01, .03}) {
    JointCommand command;
    command.action = Action::Drive;
    command.wheel_speed_targets = {.2 + residual, .2 - residual, .2 - residual, .2 + residual};
    command.body_command = {.2, 0, 0};
    check(
      ModeExecutor(c).update(command, state).feedback.fault == (residual > .02),
      "per-module residual allowance must be explicit and bounded");
  }
  c.drive_kinematic_tolerance_mps = 0;
  JointCommand exact;
  exact.action = Action::Drive;
  exact.wheel_speed_targets.fill(.2);
  exact.body_command = {.2, 0, 0};
  check(
    !ModeExecutor(c).update(exact, state).feedback.fault,
    "zero module tolerance must permit ideal rigid-body commands");
}
void test_curved_ackermann_and_spin_drive()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.minimum_mode_dwell_s = 100;
  c.samples_per_branch = 8;
  c.noise_v_mps = c.noise_w_radps = 0;
  for (auto mode : {DriveMode::DualAckermann, DriveMode::Spin}) {
    detail::Planner controller(c);
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
      check(
        !execution.feedback.fault,
        "valid curved Ackermann and Spin commands must execute "
        "despite steering tracking tolerance");
      actuate(input.vehicle, execution, c);
    }
    check(
      drove && (mode == DriveMode::Spin ? input.vehicle.pose.yaw > .3 : input.vehicle.pose.x > .2),
      "stable mode execution must make curved/rotational progress");
  }
}
void test_rollout_entry_direction()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  c.horizon_steps = 32;
  c.minimum_mode_dwell_s = 0;
  VehicleState s;
  for (std::size_t switch_step : {0u, 3u}) {
    std::vector<Control> controls(c.horizon_steps, {.4, 0, 0});
    controls[switch_step] = intent(DriveMode::Crab);
    auto trajectory = RolloutEngine(c).generate(s, {DriveMode::Crab, switch_step, true}, controls);
    check(trajectory.valid, "direction-aware switch rollout must be feasible");
    auto confirmed = s;
    for (std::size_t i = 0; i < switch_step; ++i) {
      confirmed = DriveModel(c).step(confirmed, controls[i], c.dt_s).state;
    }
    auto resume = switch_step;
    check(
      TransitionModel(c).rollout(
        confirmed, DriveMode::Crab, resume, c.horizon_steps, nullptr, controls[switch_step]) >= 0,
      "entry transition must fit the horizon");
    const auto drive = DriveModel(c).step(confirmed, controls[switch_step], c.dt_s);
    check(
      !drive.aligning && !trajectory.active_controls[resume] &&
        trajectory.controls[resume].vx == 0 && trajectory.controls[resume].vy > 0 &&
        std::abs(trajectory.poses[resume + 1].x - drive.state.pose.x) < 1e-9 &&
        std::abs(trajectory.poses[resume + 1].y - drive.state.pose.y) < 1e-9,
      "first Drive must use frozen entry intent, ignoring a different "
      "resume proposal");
  }
}
void test_capture_alignment_commitment()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  for (bool corner : {false, true}) {
    detail::Planner controller(c);
    ControllerInput in;
    in.vehicle.actual_mode = DriveMode::Crab;
    in.vehicle.time_in_mode_s = 2;
    in.vehicle.stamp_s = 1;
    in.reference_path = corner ? std::vector<Pose2d>{{0, 0, 0}, {0, .15, 0}, {.5, .15, 0}}
                               : std::vector<Pose2d>{{0, 0, 0}, {0, .1, 0}};
    const auto first = controller.compute(in);
    check(
      first.control_policy == ControlPolicy::Capture && first.action == Action::Hold,
      "terminal/corner capture must begin stopped same-mode alignment");
    in.vehicle.pose.x = .01;
    in.vehicle.stamp_s += c.dt_s;
    const auto retry = controller.compute(in);
    check(
      retry.control_policy == ControlPolicy::Alignment && retry.action == Action::Hold &&
        retry.steering_targets == first.steering_targets,
      "capture must retain its steering intent despite measured pose jitter");
    in.vehicle.stamp_s = 1 + c.confirmation_timeout_s + c.dt_s;
    const auto timeout = controller.compute(in);
    check(
      timeout.action == Action::SafeStop &&
        timeout.failure_reason == FailureReason::TransitionFault,
      "capture alignment must honor the same execution deadline as tracking");
  }
  detail::Planner controller(c);
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 0}, {0, .1, 0}};
  const auto request = controller.compute(in);
  check(
    request.mode_request && request.requested_mode == DriveMode::Crab,
    "terminal lateral capture must request Crab entry");
  in.vehicle.actual_mode = DriveMode::Crab;
  in.vehicle.mode_request_id = request.mode_request->id;
  in.vehicle.accepted_mode_request = request.mode_request;
  in.vehicle.steering_angles = request.steering_targets;
  in.vehicle.stamp_s += c.dt_s;
  check(controller.compute(in).action == Action::Hold, "confirmation must retain stopped handover");
  in.vehicle.pose.x = .01;
  in.vehicle.stamp_s += c.dt_s;
  const auto drive = controller.compute(in);
  check(
    drive.action == Action::Drive && drive.body_command.vy > 0 &&
      std::abs(drive.body_command.vx) < 1e-9,
    "capture cannot overwrite agreed mode-entry intent before its first "
    "Drive");
}

void test_stable_feedback_and_interior_speed()
{
  Config c;
  c.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  for (auto mode : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
    for (bool lost_confirmation : {false, true}) {
      VehicleState s;
      s.actual_mode = mode;
      s.stamp_s = 1;
      s.time_in_mode_s = 2;
      const Control u = mode == DriveMode::Spin ? Control{0, 0, .05} : Control{.05, 0, 0};
      const auto wheels = Kinematics(c).inverse(u, {});
      s.steering_angles = wheels.angles;
      ModeExecutor executor(c, mode);
      JointCommand hold;
      hold.action = Action::Hold;
      hold.steering_targets = s.steering_angles;
      check(
        !executor.update(hold, s).feedback.fault, "startup Hold must establish Stable execution");
      s.stamp_s += c.dt_s;
      if (lost_confirmation) {
        s.mode_confirmed = false;
      } else {
        s.actual_mode = mode == DriveMode::Spin ? DriveMode::Crab : DriveMode::Spin;
      }
      JointCommand drive;
      drive.action = Action::Drive;
      drive.requested_mode = mode;
      drive.body_command = {u.vx, u.vy, u.wz};
      drive.steering_targets = wheels.angles;
      drive.wheel_speed_targets = wheels.speeds;
      const auto rejected = executor.update(drive, s);
      check(
        rejected.feedback.fault && rejected.action == Action::SafeStop,
        "fresh unconfirmed or mismatched Stable feedback must latch a "
        "Drive fault");
    }
  }
  for (int geometry = 0; geometry < 4; ++geometry) {
    VehicleState s;
    s.stamp_s = 1;
    s.time_in_mode_s = 2;
    WheelCommand target;
    Control intent;
    if (geometry == 0) {
      s.steering_angles.fill(.1);
      s.wheel_speeds.fill(c.max_vx_mps / std::cos(.1));
      intent = {c.max_vx_mps, 0, 0};
      target = Kinematics(c).inverse(intent, s.steering_angles);
    } else if (geometry == 1) {
      auto initial = Kinematics(c).inverse({c.max_vx_mps, 0, .2}, {});
      s.steering_angles = initial.angles;
      s.wheel_speeds = initial.speeds;
      intent = {c.max_vx_mps, 0, .3};
      target = Kinematics(c).inverse(intent, s.steering_angles);
    } else if (geometry == 3) {
      s.actual_mode = DriveMode::Spin;
      const auto initial = Kinematics(c).inverse({0, 0, c.max_spin_radps}, {});
      for (std::size_t i = 0; i < 4; ++i) {
        s.steering_angles[i] = initial.angles[i] - .03;
        s.wheel_speeds[i] = initial.speeds[i] / std::cos(.03);
        target.angles[i] = initial.angles[i] + .03;
        target.speeds[i] = s.wheel_speeds[i];
      }
      intent = {0, 0, c.max_spin_radps};
    } else {
      s.actual_mode = DriveMode::Crab;
      for (std::size_t i = 0; i < 4; ++i) {
        s.steering_angles[i] = (i % 2 == 0 ? .03 : -.03);
        s.wheel_speeds[i] = c.max_crab_speed_mps / std::cos(.03);
        target.angles[i] = -s.steering_angles[i];
        target.speeds[i] = s.wheel_speeds[i];
      }
      intent = {c.max_crab_speed_mps, 0, 0};
    }
    s.velocity = Kinematics(c).forward(s.wheel_speeds, s.steering_angles);
    JointCommand drive;
    drive.action = Action::Drive;
    drive.requested_mode = s.actual_mode;
    drive.steering_targets = target.angles;
    drive.wheel_speed_targets = target.speeds;
    drive.body_command = Kinematics(c).forward(target.speeds, target.angles);
    const double limit = geometry == 2   ? c.max_crab_speed_mps
                         : geometry == 3 ? c.max_spin_radps
                                         : c.max_vx_mps;
    auto peak = [&](const std::array<double, 4> & speeds, const std::array<double, 4> & angles) {
      double maximum = 0;
      for (int step = 0; step <= 10000; ++step) {
        const double f = step / 10000.0;
        double vx = 0, vy = 0, wz = 0;
        const double x[] = {
          c.wheelbase_m / 2, c.wheelbase_m / 2, -c.wheelbase_m / 2, -c.wheelbase_m / 2};
        const double y[] = {c.track_m / 2, -c.track_m / 2, c.track_m / 2, -c.track_m / 2};
        for (std::size_t i = 0; i < 4; ++i) {
          const double v = s.wheel_speeds[i] + f * (speeds[i] - s.wheel_speeds[i]);
          const double a = s.steering_angles[i] + f * (angles[i] - s.steering_angles[i]);
          vx += v * std::cos(a) / 4;
          vy += v * std::sin(a) / 4;
          wz += (x[i] * v * std::sin(a) - y[i] * v * std::cos(a)) /
                (c.wheelbase_m * c.wheelbase_m + c.track_m * c.track_m);
        }
        maximum = std::max(
          maximum, geometry == 2   ? std::hypot(vx, vy)
                   : geometry == 3 ? std::abs(wz)
                                   : std::abs(vx));
      }
      return maximum;
    };
    check(
      peak(target.speeds, target.angles) > limit + 1e-5,
      "independent dense encoder oracle must expose the endpoint-only "
      "overspeed");
    check(
      ModeExecutor(c, s.actual_mode).update(drive, s).feedback.fault,
      "executor must reject an interior absolute-speed peak despite valid "
      "endpoints");
    const auto bounded = DriveModel(c).step(s, intent, c.dt_s);
    check(
      bounded.valid && peak(bounded.wheel_speed_targets, bounded.steering_targets) <= limit + 1e-9,
      "model Drive must satisfy absolute speeds throughout its complete "
      "joint interpolation");
    if (geometry != 2) {
      check(
        bounded.steering_targets != s.steering_angles,
        "speed limiting must preserve progress toward the requested "
        "steering geometry");
    }
    drive.body_command = bounded.state.velocity;
    drive.steering_targets = bounded.steering_targets;
    drive.wheel_speed_targets = bounded.wheel_speed_targets;
    check(
      !ModeExecutor(c, s.actual_mode).update(drive, s).feedback.fault,
      "executor must accept the model's bounded complete-period target");
  }
}

}  // namespace
int main()
{
  try {
    test_stable_feedback_and_interior_speed();
    test_all_directed_transitions();
    test_stale_ack_and_measured_alignment();
    test_executor_idempotency_and_timeout();
    test_cancellation_recovery_and_invalid_commands();
    test_request_geometry_and_id_exhaustion();
    test_persistent_mode_and_unconfirmed_drive();
    test_boot_age_drive_consistency_and_preemption();
    test_transition_prediction_deadline();
    test_transition_protocol_tick_matrix();
    test_zero_delay_controller_capture_timing();
    test_confirmation_receipt_deadline();
    test_decimal_deadline_and_tick_boundaries();
    test_same_mode_alignment_decimal_deadline();
    test_controller_executor_lateral_loop();
    test_measured_steering_tolerance();
    test_nonfinite_drive_commands();
    test_frozen_controller_alignment();
    test_drive_steering_command_limits();
    test_module_velocity_residuals();
    test_absolute_speed_and_drive_interpolation_limits();
    test_curved_ackermann_and_spin_drive();
    test_rollout_entry_direction();
    test_capture_alignment_commitment();
    std::cout << "Execution regressions passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
