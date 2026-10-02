#include "behavior_fixture.hpp"
#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/feedback.hpp"
#include "swerve_mppi/profile_runner.hpp"
#include <algorithm>
#include <iostream>

using namespace swerve_mppi;
using namespace swerve_mppi::test;
namespace {
ControllerInput input() {
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 0}, {2, 0, 0}};
  return in;
}
CommandEnvelope envelope(const ControllerInput &in, const Output &out, std::uint64_t sequence = 1,
                         std::uint64_t session = 1) {
  const double now = in.vehicle.stamp_s;
  return {session, sequence, now, out, now, now, now + .025, CommandTask::capture(in)};
}
void test_inconsistent_feedback_cannot_certify_stopping() {
  Config c;
  for (int kind = 0; kind < 9; ++kind) {
    auto in = input();
    if (kind == 0)
      in.vehicle.velocity.vx = .8; // Reviewed odometry-moving/encoders-zero case.
    if (kind == 1)
      in.vehicle.wheel_speeds.fill(.4);
    if (kind == 2)
      in.vehicle.velocity.wz = .8;
    if (kind == 3) {
      in.vehicle.actual_mode = DriveMode::Crab;
      in.vehicle.steering_angles.fill(std::acos(-1.0) / 2);
      in.vehicle.wheel_speeds.fill(.2);
      in.vehicle.velocity.vx = .2; // Wrong body-frame direction.
    }
    if (kind == 4) {
      in.vehicle.velocity.vx = .04; // Within diagnostics, above stopped threshold.
      c.collision_margin_m = 0;
      in.obstacles = {{c.robot_radius_m + .0005, 0, 0}};
    }
    if (kind == 5) {
      in.vehicle.wheel_speeds.fill(.2);
      in.vehicle.velocity.vx = .24; // Moving encoders do not cover extra body motion.
    }
    if (kind == 6)
      in.vehicle.velocity.wz = .09;
    if (kind == 7) {
      in.vehicle.actual_mode = DriveMode::Crab;
      in.vehicle.steering_angles.fill(std::acos(-1.0) / 2);
      in.vehicle.wheel_speeds.fill(.02);
      in.vehicle.velocity.vx = .02;
    }
    if (kind == 8)
      in.vehicle.velocity.vx = .001; // Also below all stopped thresholds.
    check(check_feedback(in.vehicle, c).status ==
              (kind < 4 ? FeedbackStatus::Inconsistent : FeedbackStatus::Valid),
          "diagnostics must retain configurable disagreement tolerances");
    check(check_model_feedback(in.vehicle, c).status == FeedbackStatus::Inconsistent,
          "nominal admission must reject even diagnostic-tolerance disagreement");
    const auto out = Controller(c).compute(in);
    check(ModeManager(c).update(in.vehicle).action == Action::SafeStop,
          "idle mode supervision must also reject inconsistent feedback");
    check(out.action == Action::SafeStop &&
              out.failure_reason == FailureReason::InconsistentFeedback,
          "inconsistent feedback cannot produce a healthy planning stop");
    Output brake;
    brake.action = Action::Brake;
    const auto raw = ModeExecutor(c, in.vehicle.actual_mode).update(brake, in.vehicle);
    check(
        raw.feedback.fault && !ActuationModel(c).plan(in.vehicle, raw) &&
            !ActuationModel(c).plan_stopping(in.vehicle, Action::Brake, in.vehicle.steering_angles),
        "neither raw supervision nor the nominal actuator model may certify a false stop");
    TimedExecutor timed(c, 1, in.vehicle.actual_mode);
    const auto guarded = timed.update(envelope(in, brake), in, 1);
    check(guarded.safety_error == ExecutionSafetyError::InconsistentFeedback &&
              guarded.execution.feedback.fault && !guarded.actuation,
          "guarded execution must latch instead of falling back through an invalid model");
    Trajectory stop;
    RolloutEngine(c).generate_stop(in.vehicle, stop);
    check(!stop.valid && !DriveModel(c).step(in.vehicle, {}, c.dt_s).valid &&
              !std::isfinite(Optimizer(c).optimize(in, {}).cost),
          "standalone nominal models and optimization must reject the reviewed disagreement");
    Trajectory supplied;
    supplied.valid = true;
    supplied.poses = {in.vehicle.pose};
    check(TrajectoryValidator(c).check(in, supplied) == TrajectoryStatus::Invalid,
          "a caller-supplied trace cannot bypass state admission");
  }
  auto residual = input().vehicle;
  residual.velocity.vx = .001;
  check(is_stopped(residual, c), "recovery case must be below stopped thresholds");
  bool rejected = false;
  try {
    ProfileRunner(c).reset(residual);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "diagnostic-valid residual must not authorize stopped profile recovery");
  rejected = false;
  try {
    ModeExecutor(c).reset(residual);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "diagnostic-valid residual must not authorize stopped executor recovery");
  auto state = input().vehicle;
  state.wheel_speeds.fill(.2);
  state.velocity = {.2 + c.feedback_linear_tolerance_mps, 0, c.feedback_angular_tolerance_radps};
  check(check_feedback(state, c).status == FeedbackStatus::Valid,
        "finite disagreement at configured inclusive tolerances must be admitted");
  check(check_model_feedback(state, c).status == FeedbackStatus::Inconsistent,
        "diagnostic admission must not authorize a nominal trajectory");
  state.velocity.vy = .01;
  check(check_feedback(state, c).status == FeedbackStatus::Inconsistent,
        "linear tolerance is a vector norm, not independent component allowances");
  state.velocity = Kinematics(c).forward(state.wheel_speeds, state.steering_angles);
  state.velocity.vx += 1e-10;
  check(check_model_feedback(state, c).status == FeedbackStatus::Valid,
        "nominal admission must allow numerical roundoff");
}
class AdvanceClock final : public TrajectoryConstraint {
public:
  explicit AdvanceClock(PlanningBudget::Clock::time_point &time) : time_(time) {}
  bool allows(const ControllerInput &, const Trajectory &) const override {
    time_ += std::chrono::milliseconds(20);
    return true;
  }

private:
  PlanningBudget::Clock::time_point &time_;
};
void test_shared_budget_and_late_result_rejection() {
  Config c;
  c.samples_per_branch = 2;
  c.iterations = 1;
  c.noise_v_mps = c.noise_w_radps = 0;
  auto now = PlanningBudget::Clock::time_point{};
  auto validator = std::make_shared<TrajectoryValidator>(c);
  validator->add(std::make_shared<AdvanceClock>(now));
  Controller controller(c, validator, [&] { return now; });
  auto in = input();
  const auto out = controller.compute(in);
  check(out.action == Action::SafeStop && out.failure_reason == FailureReason::ComputeTimeout &&
            out.planning_stats.budget_exhausted && out.planning_stats.branches == 1 &&
            out.planning_stats.evaluated_rollouts == 4 && !out.mode_request && !out.goal_reached,
        "all branches must share a deadline; no late partial solution may become Drive");
  now = PlanningBudget::Clock::time_point{};
  PlanningBudget budget(.01, [&] { return now; });
  now += std::chrono::milliseconds(10);
  const auto expired = Optimizer(c).optimize(in, {}, &budget);
  check(!std::isfinite(expired.cost) && expired.controls.empty() && !expired.trajectory.valid &&
            expired.planning_stats.budget_exhausted &&
            expired.planning_stats.evaluated_rollouts == 0,
        "an already expired solve must not start a rollout");
  // Terminal Hold has no optimization; the final publication gate still applies.
  now = PlanningBudget::Clock::time_point{};
  c.compute_budget_ratio = .1;
  auto terminal_validator = std::make_shared<TrajectoryValidator>(c);
  terminal_validator->add(std::make_shared<AdvanceClock>(now));
  in.reference_path = {{0, 0, 0}};
  const auto late_hold = Controller(c, terminal_validator, [&] { return now; }).compute(in);
  check(late_hold.action == Action::SafeStop &&
            late_hold.failure_reason == FailureReason::ComputeTimeout,
        "a late terminal check must not publish an expired Hold or completion");
  PlanningBudget unlimited(0, [] {
    throw std::runtime_error("disabled clock was called");
    return PlanningBudget::Clock::time_point{};
  });
  check(!unlimited.expired(), "explicit offline mode must not consult the clock");
}
void test_workload_admission_and_configuration_caps() {
  Config c;
  auto in = input();
  in.obstacles.resize(500, {100, 100, .05});
  const auto out = Controller(c).compute(in);
  check(out.action == Action::SafeStop && out.failure_reason == FailureReason::WorkloadExceeded &&
            out.planning_stats.evaluated_rollouts == 0,
        "reviewed 500-obstacle stress must be rejected before optimization, never truncated");
  in.obstacles.clear();
  in.reference_path.resize(c.max_path_points + 1);
  check(Controller(c).compute(in).failure_reason == FailureReason::WorkloadExceeded,
        "oversize paths must be rejected before scanning or progress mutation");
  for (int field = 0; field < 7; ++field) {
    auto bad = c;
    if (field == 0)
      bad.horizon_steps = 513;
    if (field == 1)
      bad.samples_per_branch = 2049;
    if (field == 2)
      bad.iterations = 33;
    if (field == 3)
      bad.stopping_horizon_steps = 4097;
    if (field == 4)
      bad.max_path_points = 0;
    if (field == 5)
      bad.max_obstacles = 0;
    if (field == 6)
      bad.compute_budget_ratio = 1.01;
    bool rejected = false;
    try {
      validate(bad);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected, "workspaces and compute budgets must have bounded validated configuration");
  }
  c.max_obstacles = 2;
  c.max_path_points = 2;
  c.compute_budget_ratio = 0; // This case tests inclusive input sizes, not host speed.
  in = input();
  in.obstacles.resize(2, {100, 100, .05});
  check(Controller(c).compute(in).action != Action::SafeStop,
        "input size boundaries must be inclusive");
}
void test_profile_sampling_watchdog_and_recovery() {
  Config c;
  c.compute_budget_ratio = 0;
  auto in = input();
  Controller controller(c);
  TimedExecutor executor(c, 1);
  const auto command = controller.compute(in);
  const auto result = executor.update(envelope(in, command), in, 1);
  check(result.actuation && result.execution.action == Action::Drive,
        "test must start with a guarded Drive profile");
  ProfileRunner runner(c);
  check(runner.install(result, 1, 10), "healthy guarded profile must install at its start");
  const auto mid = runner.sample(1.05, 10.05);
  check(mid && std::abs(mid->wheel_angular_speeds[0] -
                        .5 * result.execution.wheel_speed_targets[0] / c.wheel_radius_m) < 1e-9,
        "runner must sample full-tick interpolation and convert m/s to joint rad/s");
  check(runner.sample(1.1, 10.1).has_value(), "rounded exact tick endpoint must remain sampleable");
  check(!runner.sample(1.10001, 10.10001) && runner.fault() && !runner.install(result, 1, 11),
        "expired Drive must latch and cannot be held or replayed");
  runner.reset(in.vehicle);
  executor.reset(in.vehicle, 2);
  const auto recovered = executor.update(envelope(in, command, 1, 2), in, 1);
  check(runner.install(recovered, 1, 20),
        "verified recovery and a new session may restart execution");
  check(!runner.sample(1, 20.50001) && runner.fault(),
        "wall watchdog must expire a profile even while simulation time is paused");
  runner.reset(in.vehicle);
  check(runner.install(recovered, 1, 30) && !runner.sample(1.01, 29.99) && runner.fault(),
        "backwards wall time must fail closed");
  runner.reset(in.vehicle);
  check(runner.install(recovered, 1, 40) && !runner.sample(.99, 40.01),
        "backwards application time must fail closed");
  auto moving = in.vehicle;
  moving.wheel_speeds.fill(.2);
  moving.velocity.vx = .2;
  bool rejected = false;
  try {
    runner.reset(moving);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected && runner.fault(), "moving recovery must preserve the actuator fault latch");
  runner.reset(in.vehicle);
  auto invalid = recovered;
  invalid.safety_error = ExecutionSafetyError::CommandRejected;
  check(!runner.install(invalid, 1, 50) && runner.fault(),
        "a rejection cannot smuggle an attached Drive profile into execution");
  runner.reset(in.vehicle);
  auto changed = in;
  changed.path_id = 2;
  TimedExecutor fallback_executor(c, 3);
  const auto fallback = fallback_executor.update(envelope(in, command, 1, 3), changed, 1);
  check(fallback.safety_error == ExecutionSafetyError::TaskMismatch && fallback.actuation &&
            runner.install(fallback, 1, 60) && runner.sample(1.05, 60.05),
        "a separately checked task-rejection stop must remain executable");
  runner.reset(in.vehicle);
  check(runner.install(recovered, 1, 70), "start boundary test");
  auto next = in;
  next.vehicle.stamp_s = 1.1;
  TimedExecutor next_executor(c, 4);
  const auto next_result = next_executor.update(envelope(next, command, 1, 4), next, 1.1);
  check(runner.install(next_result, 1.1, 70.1),
        "the next checked profile may replace the previous one at the exact boundary");
  runner.reset(in.vehicle);
  check(runner.install(recovered, 1, 80), "start missed boundary test");
  check(!runner.install(next_result, 1.1, 80.50001) && runner.fault(),
        "install cannot renew a missed wall watchdog");
  runner.reset(in.vehicle);
  check(runner.install(recovered, 1, 90), "start missed tick test");
  next.vehicle.stamp_s = 1.2;
  TimedExecutor late_executor(c, 5);
  const auto late = late_executor.update(envelope(next, command, 1, 5), next, 1.2);
  check(late.actuation && !runner.install(late, 1.2, 90.2) && runner.fault(),
        "a fresh new result cannot conceal a missed profile boundary");
}

void test_pending_transition_fallback_profiles() {
  Config c;
  c.compute_budget_ratio = 0;
  for (bool task_mismatch : {true, false}) {
    auto in = input();
    class RejectOnce final : public TrajectoryConstraint {
    public:
      mutable bool reject = false;
      bool allows(const ControllerInput &, const Trajectory &) const override {
        const bool allowed = !reject;
        reject = false;
        return allowed;
      }
    };
    auto validator = std::make_shared<TrajectoryValidator>(c);
    auto reject_once = std::make_shared<RejectOnce>();
    if (!task_mismatch)
      validator->add(reject_once);
    TimedExecutor executor(c, 1, DriveMode::DualAckermann, {}, validator);
    ProfileRunner runner(c);
    Output request;
    request.action = Action::RequestMode;
    request.requested_mode = DriveMode::Crab;
    request.mode_request = ModeRequest{
        17, DriveMode::Crab, DriveModel(c).steering_for_entry(DriveMode::Crab, {0, .3, 0}, {})};
    request.steering_targets = request.mode_request->steering_targets;
    auto result = executor.update(envelope(in, request), in, in.vehicle.stamp_s);
    check(result.actuation && runner.install(result, 1, 10), "start pending alignment");
    // Reject one candidate preview; its independently rechecked fallback is safe.
    reject_once->reject = !task_mismatch;
    const auto frozen = request.mode_request->steering_targets;
    bool confirmed = false;
    for (std::uint64_t tick = 1; tick < 30; ++tick) {
      in.vehicle = result.actuation->endpoint().state; // Nominal plant regression only.
      in.vehicle.stamp_s = 1 + tick * c.dt_s;
      auto latest = in;
      if (task_mismatch)
        latest.path_id = 2;
      result = executor.update(envelope(in, request, tick + 1), latest, in.vehicle.stamp_s);
      if (tick == 1)
        check(result.safety_error == (task_mismatch ? ExecutionSafetyError::TaskMismatch
                                                    : ExecutionSafetyError::CommandRejected) &&
                  result.execution.action == Action::RequestMode,
              "reviewed rejection must preserve the active alignment fallback");
      check(result.actuation && !result.execution.feedback.fault &&
                result.execution.feedback.request_id == 17 &&
                result.execution.steering_targets == frozen &&
                result.execution.wheel_speed_targets == std::array<double, 4>{} &&
                runner.install(result, in.vehicle.stamp_s, 10 + tick * c.dt_s) &&
                runner.sample(in.vehicle.stamp_s + .05, 10 + tick * c.dt_s + .05),
            "checked pending fallback must sample with frozen identity/geometry and zero drive");
      if (result.execution.feedback.confirmed) {
        check(result.execution.action == Action::Hold &&
                  result.execution.feedback.actual_mode == DriveMode::Crab,
              "original request must complete a stopped handover");
        confirmed = true;
        break;
      }
    }
    check(confirmed, "checked fallback must permit original mode confirmation");
    check(!runner.sample(in.vehicle.stamp_s + c.dt_s + .001, 20) && runner.fault(),
          "fallback permission must not weaken expiry");
  }
  // A task mismatch may not extend a pending request's fixed deadline.
  auto in = input();
  TimedExecutor executor(c, 2);
  Output request;
  request.action = Action::RequestMode;
  request.requested_mode = DriveMode::Crab;
  request.mode_request = ModeRequest{1, DriveMode::Crab, {1, 1, 1, 1}};
  auto result = executor.update(envelope(in, request, 1, 2), in, 1);
  for (std::uint64_t tick = 1; tick < 40 && !result.execution.feedback.fault; ++tick) {
    in.vehicle.stamp_s = 1 + tick * c.dt_s; // Deliberately stalled steering.
    auto latest = in;
    latest.path_id = 2;
    result = executor.update(envelope(in, request, tick + 1, 2), latest, in.vehicle.stamp_s);
  }
  check(result.execution.feedback.fault && !result.actuation,
        "task rejection retries cannot renew the original alignment deadline");
}
void test_injected_validator_geometry() {
  Config c;
  c.compute_budget_ratio = 0;
  c.wheelbase_m = 1.2;
  c.track_m = 1;
  for (int dimension = 0; dimension < 2; ++dimension) {
    auto foreign = c;
    if (dimension == 0)
      foreign.wheelbase_m /= 2;
    else
      foreign.track_m /= 2;
    auto validator = std::make_shared<TrajectoryValidator>(foreign);
    for (int consumer = 0; consumer < 4; ++consumer) {
      bool rejected = false;
      try {
        if (consumer == 0)
          Controller controller(c, validator);
        if (consumer == 1)
          Optimizer optimizer(c, validator);
        if (consumer == 2)
          CriticManager critics(c, validator);
        if (consumer == 3)
          TimedExecutor executor(c, 1, DriveMode::Spin, {}, validator);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      check(rejected, "each validator consumer must reject foreign FK geometry at construction");
    }
  }
  auto in = input();
  in.vehicle.actual_mode = DriveMode::Spin;
  const auto wheels = Kinematics(c).inverse({0, 0, .6}, {});
  in.vehicle.wheel_speeds = wheels.speeds;
  in.vehicle.steering_angles = wheels.angles;
  in.vehicle.velocity = Kinematics(c).forward(wheels.speeds, wheels.angles);
  auto validator = std::make_shared<TrajectoryValidator>(c);
  Output brake;
  brake.action = Action::Brake;
  TimedExecutor executor(c, 1, DriveMode::Spin, {}, validator);
  const auto result = executor.update(envelope(in, brake), in, 1);
  check(result.actuation && !result.execution.feedback.fault,
        "matching custom geometry must admit and validate a spinning stop");
}
class NarrowFirstYaw final : public TrajectoryConstraint {
public:
  bool allows(const ControllerInput &, const Trajectory &trace) const override {
    if (trace.poses.size() < 2)
      return false;
    const bool stationary = std::all_of(trace.poses.begin(), trace.poses.end(), [&](const auto &p) {
      return p.x == trace.poses.front().x && p.y == trace.poses.front().y &&
             p.yaw == trace.poses.front().yaw;
    });
    return stationary || std::abs(trace.poses[1].yaw - 1e-5) < 5e-6;
  }
};
void test_retry_exploration_and_explicit_reset() {
  Config c;
  c.compute_budget_ratio = 0;
  c.random_seed = 42;
  c.minimum_mode_dwell_s = 1000;
  auto validator = std::make_shared<TrajectoryValidator>(c);
  validator->add(std::make_shared<NarrowFirstYaw>());
  Controller controller(c, validator);
  auto in = input();
  in.reference_path = {{0, 0, 0}, {3, 0, 0}};
  const auto first = controller.compute(in);
  check(first.action == Action::Hold && first.failure_reason == FailureReason::NoFeasiblePlan,
        "fixed seed must initially miss the narrow feasible band");
  auto recover = [&]() {
    for (int tick = 1; tick <= 100; ++tick) {
      in.vehicle.stamp_s = 1 + tick * c.dt_s;
      const auto out = controller.compute(in);
      if (out.action == Action::Drive)
        return std::make_pair(tick, out);
      check(out.action == Action::Hold && out.failure_reason == FailureReason::NoFeasiblePlan,
            "failed retries must remain checked stops");
    }
    throw std::runtime_error("retry exploration repeated the failed noise sequence");
  };
  const auto recovered = recover();
  controller.reset();
  in.vehicle.stamp_s = 1;
  const auto replay = controller.compute(in);
  check(replay.action == first.action && replay.failure_reason == first.failure_reason &&
            replay.planning_stats.evaluated_rollouts == first.planning_stats.evaluated_rollouts,
        "explicit controller reset must reproduce the initial failed search");
  const auto repeated = recover();
  check(repeated.first == recovered.first &&
            repeated.second.wheel_speed_targets == recovered.second.wheel_speed_targets &&
            repeated.second.steering_targets == recovered.second.steering_targets,
        "explicit reset must reproduce the entire subsequent recovery sequence");
}
} // namespace
int main() {
  try {
    test_inconsistent_feedback_cannot_certify_stopping();
    test_shared_budget_and_late_result_rejection();
    test_workload_admission_and_configuration_caps();
    test_profile_sampling_watchdog_and_recovery();
    test_pending_transition_fallback_profiles();
    test_injected_validator_geometry();
    test_retry_exploration_and_explicit_reset();
    std::cout << "Admission, compute-budget and profile-runner regressions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
