#include "swerve_mppi/controller.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace swerve_mppi;
namespace {
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
ControllerInput input() {
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.reference_path = {{0, 0, 0}, {1, 0, 0}};
  return in;
}
void test_hysteresis_selection() {
  Config c;
  c.switch_hysteresis = .4;
  ModeScheduler scheduler(c);
  Solution keep;
  keep.cost = 5;
  keep.controls = {{.3, 0, .1}};
  Solution rejected;
  rejected.branch = {DriveMode::Crab, 0, true};
  rejected.cost = 4.9;
  check(scheduler.select({keep, rejected}) == 0,
        "switch rejected by hysteresis must fall back to the keep-mode solution");
  rejected.cost = 4;
  check(scheduler.select({keep, rejected}) == 1, "sufficient improvement must permit switching");
  c.horizon_steps = 32;
  c.minimum_mode_dwell_s = 0;
  c.switch_cost = .05;
  c.switch_hysteresis = 1000;
  auto in = input();
  in.reference_path = {{0, 0, 0}, {0, 1.4, 0}};
  const auto out = Controller(c).compute(in);
  check(out.selected_cost == out.keep_cost && std::isfinite(out.keep_cost),
        "controller must execute the keep solution when mode switching is rejected");
}
void test_rollout_and_swept_collision() {
  Config c;
  c.minimum_mode_dwell_s = 0;
  RolloutEngine rollout(c);
  auto in = input();
  auto trajectory = rollout.generate(in.vehicle, {DriveMode::DualAckermann, 0, false},
                                     std::vector<Control>(c.horizon_steps, {.3, 0, 0}));
  check(trajectory.valid && trajectory.poses.size() == c.horizon_steps + 1,
        "rollouts must expose a complete timestamp-aligned pose horizon");
  CriticManager critics(c);
  check(std::isfinite(critics.score(in, trajectory)), "empty world must permit rollout");
  in.obstacles = {{trajectory.poses.back().x, 0, .01}};
  check(!std::isfinite(critics.score(in, trajectory)),
        "collision must reject the entire trajectory");
  // A large segment crossing an obstacle must be rejected even with clear endpoints.
  trajectory.poses = {{-2, 0, 0}, {2, 0, 0}};
  trajectory.final_state.pose = trajectory.poses.back();
  in.obstacles = {{0, 0, .1}};
  check(!std::isfinite(critics.score(in, trajectory)),
        "swept segments must not tunnel through obstacles");
  in.obstacles.clear();
  auto switched = rollout.generate(in.vehicle, {DriveMode::Crab, 0, true},
                                   std::vector<Control>(c.horizon_steps, {0, .3, 0}));
  check(switched.valid && !switched.active_controls[0],
        "transition ticks must be masked from control-noise updates");
}
void test_effective_noise_and_disabled_noise() {
  Config c;
  c.noise_v_mps = 1;
  c.noise_w_radps = 1;
  NoiseGenerator noise(c);
  DriveModel model(c);
  std::vector<Control> mean(c.horizon_steps, {.8, 0, .7}), candidate, effective;
  noise.sample(mean, {DriveMode::DualAckermann, 0, false}, DriveMode::DualAckermann, candidate,
               effective);
  for (std::size_t i = 0; i < mean.size(); ++i) {
    check(std::abs(mean[i].vx + effective[i].vx - candidate[i].vx) < 1e-9,
          "MPPI updates must use the effective projected perturbation");
    check(model.feasible(candidate[i], DriveMode::DualAckermann) && candidate[i].vy == 0,
          "sampling must preserve each mode's admissible controls");
  }
  c.noise_v_mps = 0;
  c.noise_w_radps = 0;
  NoiseGenerator disabled(c);
  disabled.sample(mean, {DriveMode::DualAckermann, 0, false}, DriveMode::DualAckermann, candidate,
                  effective);
  check(disabled.correction(mean, effective, std::vector<bool>(mean.size(), true)) == 0,
        "zero noise must not divide by zero");
}
void test_optimizer_reset_and_closed_loop() {
  Config c;
  c.minimum_mode_dwell_s = 100;
  c.noise_v_mps = 0;
  c.noise_w_radps = 0;
  Optimizer optimizer(c);
  auto in = input();
  auto a = optimizer.optimize(in, {DriveMode::DualAckermann, 0, false});
  optimizer.reset();
  auto b = optimizer.optimize(in, {DriveMode::DualAckermann, 0, false});
  check(std::isfinite(a.cost) && a.cost == b.cost && a.controls[0].vx == b.controls[0].vx,
        "reset must provide reproducible finite optimization");
  Controller controller(c);
  const double initial = in.reference_path.back().x;
  for (int i = 0; i < 30; ++i) {
    const auto out = controller.compute(in);
    check(out.action == Action::Drive, "aligned straight path must issue drive commands");
    // Apply the actuator targets actually emitted by the controller as feedback.
    in.vehicle.wheel_speeds = out.wheel_speed_targets;
    in.vehicle.velocity = out.body_command;
    in.vehicle.pose.x += out.body_command.vx * c.dt_s;
    in.vehicle.stamp_s += c.dt_s;
    in.vehicle.time_in_mode_s += c.dt_s;
  }
  check(std::abs(initial - in.vehicle.pose.x) < initial * .5,
        "deterministic closed-loop core must make measurable progress");
}
class RejectAll final : public Critic {
public:
  std::string_view name() const override { return "RejectAll"; }
  double score(const ControllerInput &, const Trajectory &) const override {
    return std::numeric_limits<double>::infinity();
  }
};
void test_extension_and_invalid_inputs() {
  Config c;
  Optimizer optimizer(c);
  optimizer.critics().add(std::make_shared<RejectAll>());
  check(!std::isfinite(optimizer.optimize(input(), {}).cost),
        "an injected critic must affect actual optimizer feasibility");
  auto in = input();
  in.reference_path.clear();
  check(!std::isfinite(Optimizer(c).optimize(in, {}).cost),
        "empty path must be safe at public optimizer boundary");
  in = input();
  in.vehicle.steering_angles[0] = 2;
  check(Controller(c).compute(in).action == Action::SafeStop,
        "invalid joint feedback must stop the controller");
}
} // namespace
int main() {
  try {
    test_hysteresis_selection();
    test_rollout_and_swept_collision();
    test_effective_noise_and_disabled_noise();
    test_optimizer_reset_and_closed_loop();
    test_extension_and_invalid_inputs();
    std::cout << "Optimizer regressions passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
