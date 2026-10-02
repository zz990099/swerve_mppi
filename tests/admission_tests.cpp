#include "behavior_fixture.hpp"
#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/feedback.hpp"
#include "swerve_mppi/profile_runner.hpp"
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
  for (int kind = 0; kind < 4; ++kind) {
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
    check(check_feedback(in.vehicle, c).status == FeedbackStatus::Inconsistent,
          "body/joint disagreement must fail admission");
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
  auto state = input().vehicle;
  state.wheel_speeds.fill(.2);
  state.velocity = {.2 + c.feedback_linear_tolerance_mps, 0, c.feedback_angular_tolerance_radps};
  check(check_feedback(state, c).status == FeedbackStatus::Valid,
        "finite disagreement at configured inclusive tolerances must be admitted");
  state.velocity.vy = .01;
  check(check_feedback(state, c).status == FeedbackStatus::Inconsistent,
        "linear tolerance is a vector norm, not independent component allowances");
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
} // namespace
int main() {
  try {
    test_inconsistent_feedback_cannot_certify_stopping();
    test_shared_budget_and_late_result_rejection();
    test_workload_admission_and_configuration_caps();
    test_profile_sampling_watchdog_and_recovery();
    std::cout << "Admission, compute-budget and profile-runner regressions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
