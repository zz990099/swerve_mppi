#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "planning/detail/planner.hpp"

using namespace swerve_mppi;
using namespace swerve_mppi::detail;

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

ControllerInput make_input()
{
  ControllerInput input;
  input.vehicle.time_in_mode_s = 2.0;
  input.vehicle.stamp_s = 1.0;
  input.reference_path = {{0.0, 0.0, 0.0}, {0.0, 1.4, 0.0}};
  return input;
}

void test_mode_kinematics()
{
  Config config;
  config.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  DriveModel model(config);
  check(
    !model.feasible({0.2, 0.1, 0.0}, DriveMode::DualAckermann),
    "Ackermann must forbid lateral velocity");
  check(
    !model.feasible({0.0, 0.0, 0.2}, DriveMode::DualAckermann),
    "Ackermann must forbid zero-radius turns");
  check(!model.feasible({0.1, 0.0, 0.2}, DriveMode::Spin), "spin must forbid translation");
  check(!model.feasible({0.0, 0.1, 0.2}, DriveMode::Crab), "crab must forbid yaw");
  check(model.feasible({0.0, 0.0, 0.5}, DriveMode::Spin), "spin must permit rotation from rest");

  VehicleState crab;
  crab.actual_mode = DriveMode::Crab;
  const auto first = model.step(crab, {0.0, 0.5, 0.0}, config.dt_s);
  check(
    first.valid && std::abs(first.state.pose.y) < 1e-9 && first.state.steering_angles[0] > 0.0,
    "lateral drive must first align the wheels while stationary");
  VehicleState rolling = first.state;
  for (int i = 0; i < 10; ++i) {
    rolling = model.step(rolling, {0.0, 0.5, 0.0}, config.dt_s).state;
  }
  check(
    rolling.pose.y > 0.0 && std::abs(rolling.pose.yaw) < 1e-9,
    "aligned crab drive must move laterally without rotation");
}

void test_transition_rollout()
{
  Config config;
  config.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  DriveModel model(config);
  TransitionModel transition(config);
  VehicleState state;
  state.velocity.vx = 0.45;
  state.wheel_speeds.fill(0.45);
  std::size_t steps = 0;
  std::vector<Pose2d> trace;
  const double duration = transition.rollout(state, DriveMode::Spin, steps, 40, &trace);
  check(
    duration > config.alignment_min_s + config.confirmation_prediction_s && steps == trace.size(),
    "transition must include braking and alignment");
  check(
    state.actual_mode == DriveMode::Spin && std::abs(state.velocity.vx) < 1e-9 &&
      state.pose.x > 0.0,
    "transition must finish stopped after physical braking");
  VehicleState truncated;
  truncated.velocity.vx = 0.45;
  truncated.wheel_speeds.fill(0.45);
  steps = 0;
  check(
    transition.rollout(truncated, DriveMode::Spin, steps, 2) < 0.0,
    "transition that cannot finish inside horizon is infeasible");
}

void test_confirmation_and_timeout()
{
  Config config;
  config.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  ModeManager manager(config);
  VehicleState observed;
  observed.stamp_s = 1.0;
  manager.begin(DriveMode::Crab, {}, observed);
  observed.velocity.vx = 0.2;
  observed.wheel_speeds.fill(0.2);
  check(manager.update(observed).action == Action::Brake, "mode change must brake first");
  observed.velocity = {};
  observed.wheel_speeds.fill(0);
  observed.stamp_s = 1.1;
  check(
    manager.update(observed).action == Action::RequestMode,
    "stopped vehicle should request the new mode");
  observed.stamp_s = 1.2;
  check(
    manager.update(observed).action == Action::RequestMode,
    "old actual mode must not be treated as confirmed");
  observed.actual_mode = DriveMode::Crab;
  observed.mode_confirmed = false;
  observed.stamp_s += .01;
  check(
    manager.update(observed).action == Action::RequestMode,
    "mode value without an acknowledgement must not permit driving");
  observed.mode_confirmed = true;
  observed.mode_request_id = 1;
  observed.accepted_mode_request = AcceptedModeRequest{1, DriveMode::Crab, {}};
  observed.stamp_s = 1.3;
  check(
    manager.update(observed).action == Action::Hold && !manager.active(),
    "confirmed switch must leave a zero-command handover cycle");

  observed.stamp_s = 2.0;
  manager.begin(DriveMode::Spin, {}, observed);
  observed.stamp_s = 2.0 + config.confirmation_timeout_s + 0.01;
  check(
    manager.update(observed).action == Action::SafeStop && manager.active(),
    "confirmation timeout must latch a safe stop");
  check(
    manager.update(observed).action == Action::SafeStop, "fault must remain latched until reset");
}

void test_mode_sampling_and_infeasibility()
{
  Config config;
  config.compute_budget_ratio = 0;  // Functional regression; budgets have separate clock tests.
  config.minimum_mode_dwell_s = 1.0;
  ModeScheduler scheduler(config);
  VehicleState fresh;
  fresh.time_in_mode_s = 0.0;
  for (const Branch & b : scheduler.make_branches(fresh)) {
    check(
      !b.switches || b.switch_step * config.dt_s >= 1.0 - 1e-9,
      "dwell time must constrain future switching");
  }
  fresh.mode_confirmed = false;
  check(scheduler.make_branches(fresh).size() == 1, "unconfirmed mode must prevent mode sampling");

  auto input = make_input();
  input.obstacles.push_back({0.0, 0.0, 0.1});
  detail::Planner controller(config);
  check(
    controller.compute(input).action == Action::SafeStop,
    "collision at start must reject every rollout");
  controller.reset();
  input.obstacles.clear();
  input.reference_path.clear();
  check(
    controller.compute(input).action == Action::SafeStop, "empty reference path must be rejected");
}

void test_controller_lateral_goal()
{
  Config config;
  config.compute_budget_ratio = 0;  // Algorithm regression, independent of host/debug speed.
  config.horizon_steps = 32;
  config.samples_per_branch = 100;
  config.minimum_mode_dwell_s = 0.0;
  config.switch_cost = 0.05;
  config.switch_hysteresis = 0.01;
  detail::Planner controller(config);
  auto input = make_input();
  const Prediction first = controller.compute(input);
  check(
    first.action == Action::RequestMode && first.requested_mode == DriveMode::Crab,
    "lateral goal should select crab and request its confirmation");
  input.vehicle.stamp_s += config.dt_s;
  check(
    controller.compute(input).action == Action::RequestMode,
    "pending switch must not issue lateral motion");
  input.vehicle.actual_mode = DriveMode::Crab;
  input.vehicle.mode_confirmed = true;
  input.vehicle.time_in_mode_s = 0.0;
  input.vehicle.mode_request_id = first.mode_request->id;
  input.vehicle.accepted_mode_request = first.mode_request;
  input.vehicle.steering_angles = first.mode_request->steering_targets;
  input.vehicle.stamp_s += config.dt_s;
  check(
    controller.compute(input).action == Action::Hold,
    "acknowledgement must include a zero-command handover");
  input.vehicle.stamp_s += config.dt_s;
  const Prediction drive = controller.compute(input);
  check(
    drive.action == Action::Drive && drive.body_command.vy > 0,
    "direction-aware confirmed entry should permit lateral motion directly");
}

}  // namespace

int main()
{
  try {
    test_mode_kinematics();
    test_transition_rollout();
    test_confirmation_and_timeout();
    test_mode_sampling_and_infeasibility();
    test_controller_lateral_goal();
    std::cout << "All swerve MPPI core tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Test failed: " << error.what() << '\n';
    return 1;
  }
}
