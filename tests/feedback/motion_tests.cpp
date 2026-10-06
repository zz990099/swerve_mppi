#include <iostream>
#include <limits>
#include <map>

#include "behavior_fixture.hpp"
#include "motion_fixture.hpp"
#include "swerve_mppi/execution/chassis_executor.hpp"
#include "swerve_mppi/execution/profile_runner.hpp"
#include "swerve_mppi/planning/controller.hpp"

using namespace swerve_mppi;
using swerve_mppi::test::check;
namespace
{
void test_observation_contract()
{
  Config c;
  MotionObserver observer(c);
  VehicleState state;
  state.stamp_s = 1;
  constexpr std::int64_t stamp = 1000000000;
  MotionObservation observed{{}, stamp, MotionSource::IndependentBody};
  auto result = observer.assess(state, stamp, observed);
  check(
    result.status == MotionStatus::NominalAgreement && result.body_stationary &&
      result.encoder_stationary,
    "independent nominal stationary observation");
  observed.source = MotionSource::EncoderDerived;
  result = observer.assess(state, stamp, observed);
  check(
    result.status == MotionStatus::CorrelatedSource && !result.body_stationary,
    "encoder-derived odometry cannot prove independent motion");
  check(observer.assess(state, stamp, {}).status == MotionStatus::Missing, "missing observation");
  observed.source = MotionSource::IndependentBody;
  ++observed.stamp_ns;
  check(
    observer.assess(state, stamp, observed).status == MotionStatus::Unsynchronized,
    "one nanosecond must reject coherence");
  // Adjacent source ns collapse in double at a large epoch; integer stamps do not.
  constexpr std::int64_t epoch = 1700000000000000000;
  state.stamp_s = std::chrono::duration<double>(std::chrono::nanoseconds(epoch)).count();
  observed.stamp_ns = epoch + 1;
  check(
    observer.assess(state, epoch, observed).status == MotionStatus::Unsynchronized,
    "large-epoch coherence must use original integer stamps");
  state.stamp_s = 1;
  observed.stamp_ns = stamp;
  for (int variant = 0; variant < 7; ++variant) {
    auto bad = observed;
    if (variant == 0) {
      bad.velocity.vx = std::numeric_limits<double>::quiet_NaN();
    }
    if (variant == 1) {
      bad.linear_error_bound_mps = -1;
    }
    if (variant == 2) {
      bad.angular_error_bound_radps = std::numeric_limits<double>::infinity();
    }
    if (variant == 3) {
      bad.source = static_cast<MotionSource>(99);
    }
    if (variant == 4) {
      bad.stamp_ns = -1;
    }
    if (variant == 5) {
      bad.velocity.vx = std::numeric_limits<double>::max();
      bad.linear_error_bound_mps = std::numeric_limits<double>::max();
    }
    if (variant == 6) {
      bad.velocity.wz = std::numeric_limits<double>::infinity();
    }
    result = observer.assess(state, stamp, bad);
    check(
      result.status == MotionStatus::Invalid && !result.body_stationary &&
        !result.encoder_stationary && !std::isfinite(result.linear_residual_upper_mps),
      "invalid observation cannot leak stationary evidence");
  }
  check(
    observer.assess(state, -1, observed).status == MotionStatus::Invalid, "invalid encoder time");
  check(
    observer.assess(state, stamp + 100, observed).status == MotionStatus::Invalid,
    "encoder metadata must describe the supplied snapshot");
  state.wheel_speeds[0] = c.max_wheel_speed_mps + 1;
  check(observer.assess(state, stamp, observed).status == MotionStatus::Invalid, "invalid joints");
  state.wheel_speeds = {};
  observed.linear_error_bound_mps = .001;
  result = observer.assess(state, stamp, observed);
  check(
    result.status == MotionStatus::BoundedDisagreement,
    "nonzero uncertainty cannot be called nominal even at zero measured residual");
  observed.linear_error_bound_mps = 0;
  observed.velocity.vx = c.feedback_linear_tolerance_mps;
  check(
    observer.assess(state, stamp, observed).status == MotionStatus::BoundedDisagreement,
    "diagnostic envelope includes its exact boundary");
  observed.velocity.vx =
    std::nextafter(c.feedback_linear_tolerance_mps, std::numeric_limits<double>::infinity());
  check(
    observer.assess(state, stamp, observed).status == MotionStatus::EnvelopeExceeded,
    "declared bound must not expand through numerical epsilon");
  observed.velocity = {};
  observed.angular_error_bound_radps = c.feedback_angular_tolerance_radps;
  check(
    observer.assess(state, stamp, observed).status == MotionStatus::BoundedDisagreement,
    "angular interval boundary");
  observed.angular_error_bound_radps = 0;
  observed.velocity.vx = .03;
  observed.linear_error_bound_mps = .01;
  result = observer.assess(state, stamp, observed);
  check(
    result.status == MotionStatus::BoundedDisagreement && !result.body_stationary &&
      result.encoder_stationary,
    "stationary proof uses velocity plus error bound");
  state.wheel_speeds = {.004, -.004, .004, -.004};
  observed.velocity = {};
  observed.linear_error_bound_mps = 0;
  check(observer.assess(state, stamp, observed).encoder_stationary, "wheel threshold");
  state.wheel_speeds = {.006, -.006, .006, -.006};
  check(
    !observer.assess(state, stamp, observed).encoder_stationary,
    "cancelling encoder velocities do not prove stopped wheels");
}
void test_nominal_gates_and_recovery()
{
  Config c;
  c.compute_budget_ratio = 0;
  auto input = test::scenario_input("straight");
  input.vehicle.velocity.vx = .001;
  MotionObservation o{input.vehicle.velocity, 1000000000, MotionSource::IndependentBody};
  check(
    MotionObserver(c).assess(input.vehicle, o.stamp_ns, o).status ==
      MotionStatus::BoundedDisagreement,
    "small residual is diagnostically bounded");
  check(
    check_feedback(input.vehicle, c).status == FeedbackStatus::Valid &&
      check_model_feedback(input.vehicle, c).status == FeedbackStatus::Inconsistent,
    "diagnostic and nominal contracts remain separate");
  Controller controller(c);
  auto output = controller.compute(input);
  check(
    !output.command && output.failure_reason == FailureReason::InconsistentFeedback,
    "planner must withhold authorization on small observed residual");
  for (bool switching : {false, true}) {
    Output command;
    command.command = ChassisCommand{};
    if (switching) {
      command.command->mode = DriveMode::Crab;
      command.command->mode_request = ModeRequest{7, DriveMode::Crab, {0, .3, 0}};
    } else {
      command.command->target_velocity.vx = .3;
    }
    TimedExecutor executor(c, 1);
    ProfileRunner runner(c);
    const auto make = [&](std::uint64_t sequence, std::uint64_t session) {
      const double t = input.vehicle.stamp_s;
      return CommandEnvelope{session, sequence, t,        command,
                             t,       t,        t + .025, CommandTask::capture(input)};
    };
    auto rejected = executor.update(make(1, 1), input, input.vehicle.stamp_s);
    check(
      !rejected.actuation && rejected.execution.feedback.fault &&
        rejected.execution.action == Action::SafeStop &&
        rejected.safety_error == ExecutionSafetyError::InconsistentFeedback &&
        rejected.execution.feedback.actual_mode == DriveMode::DualAckermann &&
        rejected.execution.feedback.request_id == 0,
      "residual motion must latch stop without committing a mode request");
    check(
      !runner.install(rejected, input.vehicle.stamp_s, 1) && runner.fault() &&
        !runner.sample(input.vehicle.stamp_s, 1),
      "no actuator profile after rejected observation");
    // A healthy encoder packet alone does not clear either fault latch.
    input.vehicle.velocity = {};
    input.vehicle.stamp_s += c.dt_s;
    auto retried = executor.update(make(2, 1), input, input.vehicle.stamp_s);
    check(
      !retried.actuation && retried.execution.feedback.fault,
      "fault persists after residual disappears");
    executor.reset(input.vehicle, 2);
    runner.reset(input.vehicle);
    auto recovered = executor.update(make(1, 2), input, input.vehicle.stamp_s);
    check(
      recovered.actuation && !recovered.execution.feedback.fault,
      "deliberate stopped recovery with renewed session");
    input.vehicle.velocity.vx = .001;
    input.vehicle.stamp_s = 1;
  }
}
void test_independent_probe()
{
  Config c;
  const auto rows = test::run_motion_probe(c);
  check(rows.size() == 720, "complete three-mode six-perturbation forty-tick matrix");
  std::map<std::string, double> maximum_error;
  std::map<std::string, int> hidden_stop;
  for (const auto & row : rows) {
    check(
      row.encoded_model_valid, "encoder-derived motion must remain nominal to expose blind spots");
    maximum_error[row.perturbation] = std::max(
      maximum_error[row.perturbation],
      std::max(row.prediction_position_error_m, std::abs(row.prediction_yaw_error_rad)));
    if (row.assessment.encoder_stationary && !row.assessment.body_stationary && row.braking) {
      ++hidden_stop[row.perturbation];
    }
    if (row.perturbation == "nominal") {
      check(
        row.assessment.status == MotionStatus::NominalAgreement && row.observed_model_valid,
        "nominal independent observation agrees");
      check(
        row.prediction_position_error_m < 1e-7 && std::abs(row.prediction_yaw_error_rad) < 1e-7,
        "independent nominal integration agrees with model");
    } else if (row.perturbation == "delayed") {
      check(
        row.assessment.status == MotionStatus::Unsynchronized && !row.observed_model_valid &&
          !row.assessment.body_stationary,
        "lagged observation is not current-state evidence");
    } else if (row.perturbation == "noise") {
      check(
        row.assessment.status == MotionStatus::BoundedDisagreement && !row.observed_model_valid,
        "bounded noise cannot authorize nominal models");
    } else if (row.perturbation == "wheel_lag") {
      check(
        row.assessment.status == MotionStatus::NominalAgreement,
        "body/encoder agreement alone cannot detect actuator prediction error");
    }
  }
  check(
    maximum_error["wheel_lag"] > .001 && maximum_error["body_lag"] > .001 &&
      maximum_error["slip"] > .001,
    "independent perturbations must expose model error");
  check(hidden_stop["body_lag"] > 0, "body inertia must expose false encoder-only stop evidence");
  // Analytical plant oracle: one-second linear ramp from rest to .4 m/s.
  Config ramp_config = c;
  ramp_config.dt_s = 1;
  test::MotionPlant plant(ramp_config, DriveMode::DualAckermann, {"nominal"});
  plant.advance({}, {.4, .4, .4, .4}, false);
  check(
    std::abs(plant.state.pose.x - .2) < 1e-12 && std::abs(plant.body_velocity.vx - .4) < 1e-12,
    "independent linear-ramp oracle");
  test::MotionPlant slipped(ramp_config, DriveMode::DualAckermann, {"slip", 0, 0, .25});
  slipped.advance({}, {.4, .4, .4, .4}, false);
  check(
    std::abs(slipped.state.pose.x - .15) < 1e-12 &&
      std::abs(slipped.state.velocity.vx - .4) < 1e-12 &&
      std::abs(slipped.body_velocity.vx - .3) < 1e-12,
    "slip changes physical travel but not encoders");
  // Closed-form first-order response to a linear velocity ramp, independent
  // of both core models and the fixture integration routine.
  const double tau = .15;
  const double exact_velocity = .4 * (1 - tau * (1 - std::exp(-1 / tau)));
  const double exact_distance = .4 * (.5 - tau + tau * tau * (1 - std::exp(-1 / tau)));
  test::MotionPlant lagged(ramp_config, DriveMode::DualAckermann, {"lag", tau});
  test::MotionPlant refined(ramp_config, DriveMode::DualAckermann, {"lag", tau});
  lagged.advance({}, {.4, .4, .4, .4}, false, 512);
  refined.advance({}, {.4, .4, .4, .4}, false, 1024);
  check(
    std::abs(refined.body_velocity.vx - exact_velocity) < 1e-6 &&
      std::abs(refined.state.pose.x - exact_distance) < 1e-6 &&
      std::abs(refined.state.pose.x - exact_distance) <
        std::abs(lagged.state.pose.x - exact_distance),
    "independent lag integration must converge to analytical ramp response");
}
}  // namespace
int main()
{
  try {
    test_observation_contract();
    test_nominal_gates_and_recovery();
    test_independent_probe();
    std::cout << "motion observation and independent perturbation checks passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
