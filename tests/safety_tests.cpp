#include "behavior_fixture.hpp"
#include "swerve_mppi/controller.hpp"
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
class TranslationLimit final : public TrajectoryConstraint {
public:
  bool allows(const ControllerInput &input, const Trajectory &trajectory) const override {
    // A fresh world constraint activates after the first measured tick.
    if (input.vehicle.stamp_s < 1.05)
      return true;
    for (const auto &pose : trajectory.poses)
      if (std::hypot(pose.x, pose.y) > .005)
        return false;
    return true;
  }
};
void test_shared_hard_constraints() {
  const auto c = deterministic();
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
} // namespace
int main() {
  try {
    test_temporary_failure_stops_and_recovers();
    test_unsafe_stopping_latches_fault();
    test_capture_checks_one_command_then_stop();
    test_shared_hard_constraints();
    std::cout << "Safety regressions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
