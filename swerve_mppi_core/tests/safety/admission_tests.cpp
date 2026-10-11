#include <algorithm>
#include <iostream>

#include "behavior_fixture.hpp"
#include "planning/detail/planner.hpp"
#include "swerve_mppi/feedback/feedback.hpp"

using namespace swerve_mppi;
using namespace swerve_mppi::detail;
using namespace swerve_mppi::test;
namespace
{
ControllerInput input()
{
  ControllerInput in;
  in.vehicle.stamp_ns = *duration_nanoseconds(1.0);
  in.planning_stamp_ns = in.vehicle.stamp_ns;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 0}, {2, 0, 0}};
  return in;
}

void test_inconsistent_feedback_cannot_certify_stopping()
{
  Config c;
  for (int kind = 0; kind < 9; ++kind) {
    auto in = input();
    if (kind == 0) {
      in.vehicle.velocity.vx = .8;  // Reviewed odometry-moving/encoders-zero case.
    }
    if (kind == 1) {
      in.vehicle.wheel_speeds.fill(.4);
    }
    if (kind == 2) {
      in.vehicle.velocity.wz = .8;
    }
    if (kind == 3) {
      in.vehicle.actual_mode = DriveMode::Crab;
      in.vehicle.steering_angles.fill(std::acos(-1.0) / 2);
      in.vehicle.wheel_speeds.fill(.2);
      in.vehicle.velocity.vx = .2;  // Wrong body-frame direction.
    }
    if (kind == 4) {
      in.vehicle.velocity.vx = .04;  // Within diagnostics, above stopped threshold.
      c.collision_margin_m = 0;
      in.obstacles = {{c.robot_radius_m + .0005, 0, 0}};
    }
    if (kind == 5) {
      in.vehicle.wheel_speeds.fill(.2);
      in.vehicle.velocity.vx = .24;  // Moving encoders do not cover extra body motion.
    }
    if (kind == 6) {
      in.vehicle.velocity.wz = .09;
    }
    if (kind == 7) {
      in.vehicle.actual_mode = DriveMode::Crab;
      in.vehicle.steering_angles.fill(std::acos(-1.0) / 2);
      in.vehicle.wheel_speeds.fill(.02);
      in.vehicle.velocity.vx = .02;
    }
    if (kind == 8) {
      in.vehicle.velocity.vx = .001;  // Also below all stopped thresholds.
    }
    check(
      check_feedback(in.vehicle, c).status ==
        (kind < 4 ? FeedbackStatus::Inconsistent : FeedbackStatus::Valid),
      "diagnostics must retain configurable disagreement tolerances");
    check(
      check_model_feedback(in.vehicle, c).status == FeedbackStatus::Inconsistent,
      "nominal admission must reject even diagnostic-tolerance disagreement");
    const auto out = detail::Planner(c).compute(in);
    check(
      ModeManager(c).update(in.vehicle).action == Action::SafeStop,
      "idle mode supervision must also reject inconsistent feedback");
    check(
      out.action == Action::SafeStop && out.failure_reason == FailureReason::InconsistentFeedback,
      "inconsistent feedback cannot produce a healthy planning stop");
    Trajectory interval;
    RolloutEngine(c).generate_stopping_interval(in.vehicle, in.vehicle.steering_angles, interval);
    check(!interval.valid, "stopping/alignment prediction must reject inconsistent feedback");
    Trajectory stop;
    RolloutEngine(c).generate_stop(in.vehicle, stop);
    check(
      !stop.valid && !DriveModel(c).step(in.vehicle, {}, c.model_period_s).valid &&
        !std::isfinite(Optimizer(c).optimize(in, {}).cost),
      "standalone nominal models and optimization must reject the reviewed "
      "disagreement");
    Trajectory supplied;
    supplied.valid = true;
    supplied.poses = {in.vehicle.pose};
    check(
      TrajectoryValidator(c).check(in, supplied) == TrajectoryStatus::Invalid,
      "a caller-supplied trace cannot bypass state admission");
  }
  auto state = input().vehicle;
  state.wheel_speeds.fill(.2);
  state.velocity = {.2 + c.feedback_linear_tolerance_mps, 0, c.feedback_angular_tolerance_radps};
  check(
    check_feedback(state, c).status == FeedbackStatus::Valid,
    "finite disagreement at configured inclusive tolerances must be "
    "admitted");
  check(
    check_model_feedback(state, c).status == FeedbackStatus::Inconsistent,
    "diagnostic admission must not authorize a nominal trajectory");
  state.velocity.vy = .01;
  check(
    check_feedback(state, c).status == FeedbackStatus::Inconsistent,
    "linear tolerance is a vector norm, not independent component "
    "allowances");
  state.velocity = Kinematics(c).forward(state.wheel_speeds, state.steering_angles);
  state.velocity.vx += 1e-10;
  check(
    check_model_feedback(state, c).status == FeedbackStatus::Valid,
    "nominal admission must allow numerical roundoff");
}
class AdvanceClock final : public TrajectoryConstraint
{
public:
  explicit AdvanceClock(PlanningBudget::Clock::time_point & time) : time_(time) {}
  bool allows(const ControllerInput &, const Trajectory &) const override
  {
    time_ += std::chrono::milliseconds(20);
    return true;
  }

private:
  PlanningBudget::Clock::time_point & time_;
};
void test_shared_budget_and_late_result_rejection()
{
  Config c;
  c.samples_per_branch = 2;
  c.iterations = 1;
  c.noise_v_mps = c.noise_w_radps = 0;
  auto now = PlanningBudget::Clock::time_point{};
  auto validator = std::make_shared<TrajectoryValidator>(c);
  validator->add(std::make_shared<AdvanceClock>(now));
  detail::Planner controller(c, validator, [&] { return now; });
  auto in = input();
  const auto out = controller.compute(in);
  check(
    out.action == Action::SafeStop && out.failure_reason == FailureReason::ComputeTimeout &&
      out.planning_stats.budget_exhausted && out.planning_stats.branches == 1 &&
      out.planning_stats.evaluated_rollouts == 4 && !out.mode_request && !out.goal_reached,
    "all branches must share a deadline; no late partial solution may "
    "become Drive");
  now = PlanningBudget::Clock::time_point{};
  PlanningBudget budget(.01, [&] { return now; });
  now += std::chrono::milliseconds(10);
  const auto expired = Optimizer(c).optimize(in, {}, &budget);
  check(
    !std::isfinite(expired.cost) && expired.controls.empty() && !expired.trajectory.valid &&
      expired.planning_stats.budget_exhausted && expired.planning_stats.evaluated_rollouts == 0,
    "an already expired solve must not start a rollout");
  // Terminal Hold has no optimization; the final publication gate still
  // applies.
  now = PlanningBudget::Clock::time_point{};
  c.compute_budget_ratio = .1;
  auto terminal_validator = std::make_shared<TrajectoryValidator>(c);
  terminal_validator->add(std::make_shared<AdvanceClock>(now));
  in.reference_path = {{0, 0, 0}};
  const auto late_hold = detail::Planner(c, terminal_validator, [&] { return now; }).compute(in);
  check(
    late_hold.action == Action::SafeStop &&
      late_hold.failure_reason == FailureReason::ComputeTimeout,
    "a late terminal check must not publish an expired Hold or completion");
  PlanningBudget unlimited(0, [] {
    throw std::runtime_error("disabled clock was called");
    return PlanningBudget::Clock::time_point{};
  });
  check(!unlimited.expired(), "explicit offline mode must not consult the clock");
}
void test_workload_admission_and_configuration_caps()
{
  Config c;
  auto in = input();
  in.obstacles.resize(500, {100, 100, .05});
  const auto out = detail::Planner(c).compute(in);
  check(
    out.action == Action::SafeStop && out.failure_reason == FailureReason::WorkloadExceeded &&
      out.planning_stats.evaluated_rollouts == 0,
    "reviewed 500-obstacle stress must be rejected before optimization, "
    "never truncated");
  in.obstacles.clear();
  in.reference_path.resize(c.max_path_points + 1);
  check(
    detail::Planner(c).compute(in).failure_reason == FailureReason::WorkloadExceeded,
    "oversize paths must be rejected before scanning or progress mutation");
  for (int field = 0; field < 7; ++field) {
    auto bad = c;
    if (field == 0) {
      bad.horizon_steps = 513;
    }
    if (field == 1) {
      bad.samples_per_branch = 2049;
    }
    if (field == 2) {
      bad.iterations = 33;
    }
    if (field == 3) {
      bad.stopping_horizon_steps = 4097;
    }
    if (field == 4) {
      bad.max_path_points = 0;
    }
    if (field == 5) {
      bad.max_obstacles = 0;
    }
    if (field == 6) {
      bad.compute_budget_ratio = 1.01;
    }
    bool rejected = false;
    try {
      validate(bad);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(
      rejected,
      "workspaces and compute budgets must have bounded validated "
      "configuration");
  }
  c.max_obstacles = 2;
  c.max_path_points = 2;
  c.compute_budget_ratio = 0;  // This case tests inclusive input sizes, not host speed.
  in = input();
  in.obstacles.resize(2, {100, 100, .05});
  check(
    detail::Planner(c).compute(in).action != Action::SafeStop,
    "input size boundaries must be inclusive");
}

void test_injected_validator_geometry()
{
  Config c;
  c.compute_budget_ratio = 0;
  c.wheelbase_m = 1.2;
  c.track_m = 1;
  for (int dimension = 0; dimension < 2; ++dimension) {
    auto foreign = c;
    if (dimension == 0) {
      foreign.wheelbase_m /= 2;
    } else {
      foreign.track_m /= 2;
    }
    auto validator = std::make_shared<TrajectoryValidator>(foreign);
    for (int consumer = 0; consumer < 3; ++consumer) {
      bool rejected = false;
      try {
        if (consumer == 0) {
          detail::Planner controller(c, validator);
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
      check(
        rejected,
        "each validator consumer must reject foreign FK geometry at "
        "construction");
    }
  }
  auto in = input();
  in.vehicle.actual_mode = DriveMode::Spin;
  const auto wheels = Kinematics(c).inverse({0, 0, .6}, {});
  in.vehicle.wheel_speeds = wheels.speeds;
  in.vehicle.steering_angles = wheels.angles;
  in.vehicle.velocity = Kinematics(c).forward(wheels.speeds, wheels.angles);
  auto validator = std::make_shared<TrajectoryValidator>(c);
  Trajectory stop;
  RolloutEngine(c).generate_stop(in.vehicle, stop);
  check(
    validator->check(in, stop) == TrajectoryStatus::Valid,
    "matching geometry must validate a spinning stopping prediction");
}
class NarrowFirstYaw final : public TrajectoryConstraint
{
public:
  bool allows(const ControllerInput &, const Trajectory & trace) const override
  {
    if (trace.poses.size() < 2) {
      return false;
    }
    const bool stationary =
      std::all_of(trace.poses.begin(), trace.poses.end(), [&](const auto & p) {
        return p.x == trace.poses.front().x && p.y == trace.poses.front().y &&
               p.yaw == trace.poses.front().yaw;
      });
    return stationary || std::abs(trace.poses[1].yaw - 1e-5) < 5e-6;
  }
};
void test_retry_exploration_and_explicit_reset()
{
  Config c;
  c.compute_budget_ratio = 0;
  c.random_seed = 42;
  c.minimum_mode_dwell_s = 1000;
  auto validator = std::make_shared<TrajectoryValidator>(c);
  validator->add(std::make_shared<NarrowFirstYaw>());
  detail::Planner controller(c, validator);
  auto in = input();
  in.reference_path = {{0, 0, 0}, {3, 0, 0}};
  const auto first = controller.compute(in);
  check(
    first.action == Action::Hold && first.failure_reason == FailureReason::NoFeasiblePlan,
    "fixed seed must initially miss the narrow feasible band");
  auto recover = [&]() {
    for (int tick = 1; tick <= 100; ++tick) {
      in.vehicle.stamp_ns = *duration_nanoseconds(1 + tick * c.model_period_s);
      in.planning_stamp_ns = in.vehicle.stamp_ns;
      const auto out = controller.compute(in);
      if (out.action == Action::Drive) {
        return std::make_pair(tick, out);
      }
      check(
        out.action == Action::Hold && out.failure_reason == FailureReason::NoFeasiblePlan,
        "failed retries must remain checked stops");
    }
    throw std::runtime_error("retry exploration repeated the failed noise sequence");
  };
  const auto recovered = recover();
  controller.reset();
  in.vehicle.stamp_ns = *duration_nanoseconds(1.0);
  in.planning_stamp_ns = in.vehicle.stamp_ns;
  const auto replay = controller.compute(in);
  check(
    replay.action == first.action && replay.failure_reason == first.failure_reason &&
      replay.planning_stats.evaluated_rollouts == first.planning_stats.evaluated_rollouts,
    "explicit controller reset must reproduce the initial failed search");
  const auto repeated = recover();
  check(
    repeated.first == recovered.first &&
      repeated.second.wheel_speed_targets == recovered.second.wheel_speed_targets &&
      repeated.second.steering_targets == recovered.second.steering_targets,
    "explicit reset must reproduce the entire subsequent recovery sequence");
}
}  // namespace
int main()
{
  try {
    test_inconsistent_feedback_cannot_certify_stopping();
    test_shared_budget_and_late_result_rejection();
    test_workload_admission_and_configuration_caps();

    test_injected_validator_geometry();
    test_retry_exploration_and_explicit_reset();
    std::cout << "Admission and compute-budget regressions passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
