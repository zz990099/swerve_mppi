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
  mean.assign(c.horizon_steps, {0, .4, 0});
  noise.sample(mean, {DriveMode::Crab, 0, true}, DriveMode::DualAckermann, candidate, effective);
  check(candidate[0].vx == mean[0].vx && candidate[0].vy == mean[0].vy && effective[0].vx == 0 &&
            effective[0].vy == 0 && effective[0].wz == 0,
        "proposals must freeze entry intent instead of hiding steering changes in masked noise");
  c.noise_v_mps = 0;
  c.noise_w_radps = 0;
  NoiseGenerator disabled(c);
  disabled.sample(mean, {DriveMode::DualAckermann, 0, false}, DriveMode::DualAckermann, candidate,
                  effective);
  check(disabled.correction(mean, effective, std::vector<bool>(mean.size(), true)) == 0,
        "zero noise must not divide by zero");
}
// Independent dense covariance solve, rather than the production recurrence.
double covariance_score(const std::vector<Control> &mean, const std::vector<Control> &delta,
                        const std::vector<bool> &active, double rho, const Branch &branch) {
  std::vector<std::size_t> indices;
  for (std::size_t i = 0; i < active.size(); ++i)
    if (active[i] && !(branch.switches && i == branch.switch_step))
      indices.push_back(i);
  const auto n = indices.size();
  std::vector<std::vector<double>> matrix(n, std::vector<double>(n + 1));
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      const auto a = indices[i], b = indices[j];
      const bool separated =
          branch.switches && ((a < branch.switch_step) != (b < branch.switch_step));
      matrix[i][j] = separated ? 0 : std::pow(rho, a > b ? a - b : b - a);
    }
    matrix[i][n] = delta[indices[i]].vx;
  }
  for (std::size_t k = 0; k < n; ++k) {
    const double pivot = matrix[k][k];
    for (std::size_t j = k; j <= n; ++j)
      matrix[k][j] /= pivot;
    for (std::size_t i = 0; i < n; ++i) {
      if (i == k)
        continue;
      const double factor = matrix[i][k];
      for (std::size_t j = k; j <= n; ++j)
        matrix[i][j] -= factor * matrix[k][j];
    }
  }
  double score = 0;
  for (std::size_t i = 0; i < n; ++i)
    score += mean[indices[i]].vx * matrix[i][n];
  return score;
}
void test_marginal_covariance() {
  Config c;
  c.noise_v_mps = 1;
  c.noise_w_radps = 0;
  c.control_correction_weight = 1;
  std::vector<Control> mean(8), delta(8);
  for (std::size_t i = 0; i < mean.size(); ++i) {
    mean[i].vx = .1 * (i + 1);
    delta[i].vx = .03 * (i % 3) - .02;
  }
  for (double rho : {0.0, .5, .85, .99}) {
    c.noise_correlation = rho;
    NoiseGenerator noise(c);
    for (const Branch branch :
         {Branch{}, Branch{DriveMode::Crab, 3, true}, Branch{DriveMode::Crab, 0, true}})
      for (unsigned mask = 0; mask < 256; ++mask) {
        std::vector<bool> active(8);
        for (unsigned i = 0; i < 8; ++i)
          active[i] = (mask & (1u << i)) != 0;
        check(std::abs(noise.correction(mean, delta, active, branch) -
                       covariance_score(mean, delta, active, rho, branch)) < 1e-9,
              "sparse correction must match inverse marginal covariance including entry resets");
      }
  }
  c.noise_correlation = .85;
  c.noise_v_mps = .8;
  c.noise_w_radps = .7;
  const std::vector<bool> mask{true, false, true, false, false, true, true, false};
  auto lateral_mean = mean, lateral_delta = delta, yaw_mean = mean, yaw_delta = delta;
  for (std::size_t i = 0; i < mean.size(); ++i) {
    lateral_mean[i].vx *= -.4;
    lateral_delta[i].vx *= .3;
    yaw_mean[i].vx *= .6;
    yaw_delta[i].vx *= -.2;
    mean[i].vy = lateral_mean[i].vx;
    delta[i].vy = lateral_delta[i].vx;
    mean[i].wz = yaw_mean[i].vx;
    delta[i].wz = yaw_delta[i].vx;
  }
  const double expected = (covariance_score(mean, delta, mask, .85, {}) +
                           covariance_score(lateral_mean, lateral_delta, mask, .85, {})) /
                              (.8 * .8) +
                          covariance_score(yaw_mean, yaw_delta, mask, .85, {}) / (.7 * .7);
  check(std::abs(NoiseGenerator(c).correction(mean, delta, mask) - expected) < 1e-9,
        "each enabled dimension must use its own marginal variance");
  c.noise_w_radps = 0;
  c.noise_correlation = .5;
  c.noise_v_mps = .01;
  NoiseGenerator generator(c);
  mean.assign(3, {.2, 0, 0});
  for (const Branch branch : {Branch{}, Branch{DriveMode::Crab, 1, true}}) {
    double sum_x = 0, sum_y = 0, xx = 0, yy = 0, xy = 0;
    constexpr int count = 50000;
    for (int i = 0; i < count; ++i) {
      std::vector<Control> candidate, effective;
      generator.sample(mean, branch, DriveMode::DualAckermann, candidate, effective);
      const double x = effective[0].vx, y = effective[2].vx;
      sum_x += x;
      sum_y += y;
      xx += x * x;
      yy += y * y;
      xy += x * y;
    }
    const double correlation =
        (xy - sum_x * sum_y / count) /
        std::sqrt((xx - sum_x * sum_x / count) * (yy - sum_y * sum_y / count));
    check(std::abs(correlation - (branch.switches ? 0 : .25)) < .02,
          "sampling must preserve gap correlation and reset only at explicit entry");
  }
  VehicleState state;
  state.actual_mode = DriveMode::Crab;
  mean.assign(c.horizon_steps, {0, .4, 0});
  const auto trace = RolloutEngine(c).generate(state, {}, mean);
  check(trace.valid && trace.active_controls[0] && !trace.active_controls[1],
        "real alignment must expose the sparse active mask");
  // Exercise another noise dimension with the real rollout mask.
  delta.assign(mean.size(), {.01, .02, .03});
  for (auto &u : mean) {
    u.vx = u.vy;
    u.vy = 0;
  }
  for (auto &u : delta) {
    u.vx = u.vy;
    u.vy = 0;
  }
  c.noise_v_mps = 1;
  NoiseGenerator alignment(c);
  check(std::abs(alignment.correction(mean, delta, trace.active_controls) -
                 covariance_score(mean, delta, trace.active_controls, .5, {})) < 1e-9,
        "alignment masks must retain the sampled marginal covariance");
}
void test_correlated_noise_precision() {
  Config c;
  c.noise_correlation = .5;
  c.noise_v_mps = 1;
  c.noise_w_radps = 0;
  c.control_correction_weight = 1;
  NoiseGenerator noise(c);
  std::vector<Control> mean{{1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  std::vector<Control> perturbation{{.2, 0, 0}, {.4, 0, 0}, {.1, 0, 0}};
  const double expected = .2 + (2 - .5) * (.4 - .1) / .75 + (3 - 1) * (.1 - .2) / .75;
  check(std::abs(noise.correction(mean, perturbation, {true, true, true}) - expected) < 1e-9,
        "correlated proposals must use their temporal precision in weighting");
  check(std::abs(noise.correction(mean, perturbation, {true, false, true}) - 26.0 / 75.0) < 1e-9,
        "masked intervals must retain marginal correlation across the gap");
  mean.assign(c.horizon_steps, {.2, 0, 0});
  std::vector<Control> a, b, effective;
  noise.sample(mean, {}, DriveMode::DualAckermann, a, effective);
  noise.reset();
  noise.sample(mean, {}, DriveMode::DualAckermann, b, effective);
  for (std::size_t i = 0; i < a.size(); ++i)
    check(a[i].vx == b[i].vx && a[i].wz == b[i].wz,
          "reset must reproduce nonzero correlated proposals");
  for (double bad : {-1.0, 1.0, std::numeric_limits<double>::quiet_NaN()}) {
    c.noise_correlation = bad;
    bool rejected = false;
    try {
      validate(c);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected, "invalid temporal correlation must be rejected");
  }
}
void test_frozen_alignment_rollout() {
  Config c;
  c.minimum_mode_dwell_s = 0;
  VehicleState state;
  state.actual_mode = DriveMode::Crab;
  std::vector<Control> controls(c.horizon_steps, {.4, 0, 0});
  controls[0] = {0, .4, 0};
  const auto trace = RolloutEngine(c).generate(state, {DriveMode::Crab, 0, false}, controls);
  check(trace.valid && trace.active_controls[0] && !trace.active_controls[1],
        "alignment entry influences rollout but following proposals are masked");
  std::size_t first = 1;
  while (first < trace.poses.size() && trace.poses[first].y == 0)
    ++first;
  check(first < trace.poses.size() && trace.poses[first].y > 0 &&
            std::abs(trace.poses[first].x) < trace.poses[first].y * .25 &&
            trace.controls[first - 1].vx == 0 && trace.controls[first - 1].vy > 0,
        "rollout must finish the frozen lateral alignment before changing direction");
  const auto interrupted = RolloutEngine(c).generate(state, {DriveMode::Spin, 1, true}, controls);
  check(!interrupted.valid, "a scheduled mode switch cannot preempt local alignment");
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
void test_reusable_rollouts_and_planning_stats() {
  Config c;
  c.minimum_mode_dwell_s = 100;
  c.noise_v_mps = c.noise_w_radps = 0;
  c.samples_per_branch = 8;
  RolloutEngine engine(c);
  auto in = input();
  std::vector<Control> controls(c.horizon_steps, {.2, 0, 0});
  Trajectory reuse;
  engine.generate(in.vehicle, {}, controls, reuse);
  auto original = reuse;
  const auto *storage = reuse.poses.data();
  engine.generate(in.vehicle, {}, controls, reuse);
  check(reuse.valid && reuse.poses.data() == storage &&
            reuse.poses.back().x == original.poses.back().x,
        "reused valid rollouts must preserve capacity and trajectory semantics");
  controls.pop_back();
  engine.generate(in.vehicle, {}, controls, reuse);
  check(!reuse.valid && reuse.poses.empty() && reuse.controls.empty() &&
            reuse.active_controls.empty(),
        "invalid reuse must clear validity and previous trace data");
  const auto result = Optimizer(c).optimize(in, {});
  const auto expected = 1 + c.iterations * (c.samples_per_branch + 1);
  check(result.planning_stats.evaluated_rollouts == expected &&
            result.planning_stats.feasible_rollouts == expected &&
            result.planning_stats.fallback_updates == 0,
        "work counters must include nominal, sampled and weighted evaluations");
  const auto output = Controller(c).compute(in);
  check(output.control_policy == ControlPolicy::Tracking && output.planning_stats.branches == 1 &&
            output.planning_stats.evaluated_rollouts == expected &&
            output.failure_reason == FailureReason::None,
        "controller diagnostics must report the full executed planning work");
  Optimizer optimizer(c);
  const auto first = optimizer.optimize(in, {});
  in.reference_path = {{0, 0, 0}, {-1, 0, 0}};
  optimizer.optimize(in, {});
  check(first.controls.front().vx > 0 && first.trajectory.poses.back().x > 0,
        "returned solutions must own data independently of reused workspace");
  auto copied = optimizer;
  optimizer.reset();
  copied.reset();
  const auto a = optimizer.optimize(in, {}), b = copied.optimize(in, {});
  check(a.cost == b.cost && a.controls.front().vx == b.controls.front().vx,
        "copied optimizers must not alias each other's workspace");
  Controller controller(c);
  in = input();
  controller.compute(in);
  check(controller.compute(in).failure_reason == FailureReason::NonmonotonicTime,
        "clock faults must have an inspectable reason");
  in.vehicle.stamp_s += c.dt_s;
  in.obstacles = {{0, 0, .1}};
  const auto blocked = controller.compute(in);
  check(blocked.action == Action::SafeStop &&
            blocked.failure_reason == FailureReason::UnsafeStoppingTrajectory &&
            blocked.planning_stats.evaluated_rollouts > 0 &&
            blocked.planning_stats.feasible_rollouts == 0 &&
            blocked.planning_stats.fallback_updates > 0,
        "blocked paths must report work and infeasibility without stale counters");
}
void test_tracking_speed_limit() {
  Config c;
  c.minimum_mode_dwell_s = 0;
  for (auto mode : {DriveMode::DualAckermann, DriveMode::Crab}) {
    auto in = input();
    in.vehicle.actual_mode = mode;
    in.tracking = TrackingContext{{1, 0, 0}, .2, .08, true, PathHeadingPolicy::FollowPath};
    const auto result = Optimizer(c).optimize(in, {mode, 0, false});
    check(std::isfinite(result.cost), "speed-limited tracking must retain feasible proposals");
    for (const auto &u : result.controls)
      check(std::hypot(u.vx, u.vy) <= .08 + 1e-9,
            "nominal, noise and weighted controls must obey terminal speed limit");
  }
  auto in = input();
  in.reference_path = {{0, 0, 0}, {0, 1, 0}};
  in.tracking = TrackingContext{{0, 1, 0}, 1, .08, false, PathHeadingPolicy::FollowPath};
  const auto result = Optimizer(c).optimize(in, {DriveMode::Crab, 0, true});
  check(std::isfinite(result.cost) && result.controls[0].vx == 0 &&
            std::abs(result.controls[0].vy - .08) < 1e-9,
        "tracking proposals must preserve the constrained frozen mode-entry seed");
}
class ObserveCurveSpeed final : public Critic {
public:
  mutable double maximum = 0;
  std::string_view name() const override { return "ObserveCurveSpeed"; }
  double score(const ControllerInput &, const Trajectory &trace) const override {
    for (const auto &u : trace.controls)
      maximum = std::max(maximum, std::hypot(u.vx, u.vy));
    return 0;
  }
};
void test_curve_yaw_budget() {
  Config c;
  c.horizon_steps = 16;
  c.samples_per_branch = 16;
  auto in = input();
  // Three equal chords of a radius-0.5 circle require speed <= yaw budget * radius.
  in.reference_path = {{0, 0, 0},
                       {.4330127018922193, .25, 1.0471975511965976},
                       {.4330127018922193, .75, 2.0943951023931953}};
  const auto original_path = in.reference_path;
  for (int repetitions : {1, 2, 5}) {
    in.reference_path.clear();
    for (const auto &point : original_path)
      for (int n = 0; n < repetitions; ++n)
        in.reference_path.push_back(point);
    for (auto mode : {DriveMode::DualAckermann, DriveMode::Crab}) {
      in.vehicle.actual_mode = mode;
      Optimizer optimizer(c);
      auto observer = std::make_shared<ObserveCurveSpeed>();
      optimizer.critics().add(observer);
      const auto result = optimizer.optimize(in, {mode, 0, false});
      check(std::isfinite(result.cost), "curved proposals must remain feasible");
      if (mode == DriveMode::DualAckermann)
        check(observer->maximum <= .5 * c.max_yaw_rate_radps + 1e-9,
              "all nominal, sampled and weighted curve proposals must anticipate the yaw budget");
      else
        check(observer->maximum > .5 * c.max_yaw_rate_radps,
              "crab translation must retain its speed budget on a curved reference");
    }
  }
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
    test_marginal_covariance();
    test_correlated_noise_precision();
    test_frozen_alignment_rollout();
    test_optimizer_reset_and_closed_loop();
    test_extension_and_invalid_inputs();
    test_tracking_speed_limit();
    test_curve_yaw_budget();
    test_reusable_rollouts_and_planning_stats();
    std::cout << "Optimizer regressions passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
