#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "behavior_fixture.hpp"
#include "swerve_mppi/model/rollout.hpp"
#include "swerve_mppi/safety/trajectory_validator.hpp"
using namespace swerve_mppi;
namespace
{
void check(bool ok, const char * message)
{
  if (!ok) throw std::runtime_error(message);
}
bool close(double a, double b) { return std::abs(a - b) < 1e-9; }
VehicleState moving(const Config & c, DriveMode mode, Control u)
{
  VehicleState s;
  s.actual_mode = mode;
  const auto w = Kinematics(c).inverse(u, {});
  s.steering_angles = w.angles;
  s.wheel_speeds = w.speeds;
  s.velocity = Kinematics(c).forward(w.speeds, w.angles);
  return s;
}
void kinematics_and_limits()
{
  Config c;
  Kinematics k(c);
  for (double a : {-c.steering_limit_rad, 0., c.steering_limit_rad}) {
    std::array<double, 4> current{a, a, a, a};
    for (Control u : {Control{-.4, 0, 0}, {0, .4, 0}, {0, 0, .5}, {.4, 0, .3}, {-.3, .2, -.4}}) {
      const auto w = k.inverse(u, current);
      const auto v = k.forward(w.speeds, w.angles);
      check(
        w.valid && close(v.vx, u.vx) && close(v.vy, u.vy) && close(v.wz, u.wz),
        "signed bounded IK/FK must round trip");
      for (double angle : w.angles)
        check(std::abs(angle) <= c.steering_limit_rad, "hard steering stop");
    }
  }
  const auto tie = k.inverse({0, .4, 0}, {});
  check(
    close(tie.angles[0], -c.steering_limit_rad) && tie.speeds[0] < 0,
    "nearest-angle ties follow current Python candidate order");
  check(!k.inverse({.2, 0, 0}, {3, 0, 0, 0}).valid, "out-of-range joints rejected");
  DriveModel model(c);
  const auto capped = model.bounded({4, 0, 2});
  check(close(capped.vx, .8) && close(capped.wz, .4), "body saturation preserves curvature");
  const auto entry = model.steering_for_entry(DriveMode::DualAckermann, {.2, 0, .7}, {});
  const auto raw = k.inverse({.2 / .7, 0, 1}, {}).angles;
  check(
    entry == raw, "entry geometry must preserve raw legal curvature without planner projection");
  c.max_wheel_speed_mps = .2;
  c.max_wheel_accel_mps2 = .1;
  const auto projected = DriveModel(c).project({4, 0, 2}, DriveMode::DualAckermann);
  check(
    DriveModel(c).feasible(projected, DriveMode::DualAckermann),
    "planner projection makes admissible proposals");
  auto s = moving(c, DriveMode::DualAckermann, {});
  s.steering_angles = Kinematics(c).inverse(projected, {}).angles;
  const auto next = DriveModel(c).step(s, projected, .1);
  check(next.valid, "bounded wheel target prediction");
  for (double v : next.wheel_speed_targets)
    check(std::abs(v) <= .01 + 1e-9, "independent wheel acceleration bound");
  for (double bad : {1., 2., 3.141592653589793}) {
    c.steering_limit_rad = bad;
    bool rejected = false;
    try {
      validate(c);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected, "only the current +/- pi/2 chassis is supported");
  }
}
void zero_and_discrete_stop()
{
  Config c;
  DriveModel model(c);
  VehicleState s;
  s.wheel_speeds = {.6, .2, -.1, .05};
  s.velocity = Kinematics(c).forward(s.wheel_speeds, s.steering_angles);
  const auto next = model.step(s, {}, .1);
  check(
    next.valid && !next.aligning && next.steering_targets == s.steering_angles,
    "ordinary zero holds steering");
  check(
    close(next.wheel_speed_targets[0], .2) && next.wheel_speed_targets[1] == 0 &&
      next.wheel_speed_targets[2] == 0 && next.wheel_speed_targets[3] == 0,
    "zero brakes wheels independently, not proportionally");
  check(next.prediction.limited_velocity.vx == 0, "zero immediately clears limited body intent");
  for (double speed : {.003, .1, .35}) {
    for (DriveMode mode : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
      const Control u = mode == DriveMode::Spin   ? Control{0, 0, speed}
                        : mode == DriveMode::Crab ? Control{0, speed, 0}
                                                  : Control{speed, 0, 0};
      const auto initial = moving(c, mode, u);
      Trajectory stop;
      RolloutEngine(c).generate_stop(initial, stop);
      double expected = 0;
      double wheel = std::abs(initial.wheel_speeds[0]);
      const double delta = c.max_wheel_accel_mps2 * c.chassis_period_s;
      for (int tick = 0; tick < 1000 && wheel > 0; ++tick) {
        expected += wheel * c.chassis_period_s;
        wheel = std::max(0., wheel - delta);
      }
      const double ratio = expected / std::abs(initial.wheel_speeds[0]);
      check(
        stop.valid && close(stop.final_state.pose.x, u.vx * ratio) &&
          close(stop.final_state.pose.y, u.vy * ratio) &&
          close(stop.final_state.pose.yaw, u.wz * ratio),
        "discrete held-target brake matches independent geometric-series oracle");
      check(
        stop.final_state.wheel_speeds == std::array<double, 4>{},
        "stop tail includes sub-threshold residual travel");
    }
  }
}
void history_and_alignment()
{
  Config c;
  DriveModel model(c);
  auto state = moving(c, DriveMode::Crab, {.4, 0, 0});
  auto memory = model.seed(state);
  memory.limited_velocity = {};  // ordinary zero cleared the slew state while wheels coast
  bool saw_alignment = false, saw_drive = false;
  for (int tick = 0; tick < 150; ++tick) {
    const auto next = model.step(state, {0, .4, 0}, c.chassis_period_s, memory);
    check(next.valid, "same-mode automatic alignment completes");
    if (next.prediction.phase == TransitionPhase::Braking)
      check(
        next.steering_targets == state.steering_angles,
        "measured rolling must brake before steering");
    if (next.prediction.phase == TransitionPhase::Aligning) {
      saw_alignment = true;
      check(
        next.wheel_speed_targets == std::array<double, 4>{},
        "steering alignment requires commanded zero wheels");
    }
    saw_drive = saw_drive || next.state.velocity.vy > .05;
    state = next.state;
    memory = next.prediction;
  }
  check(
    saw_alignment && saw_drive && state.pose.y > 0,
    "persistent prediction resumes frozen entry after realignment");
  auto delayed = moving(c, DriveMode::Crab, {.1, 0, 0});
  const auto target = model.steering_for_entry(DriveMode::Crab, {0, .4, 0}, {});
  memory = model.alignment_seed(delayed, target);
  memory.commanded_wheel_radps.fill(0);  // target zero, measured wheels still rolling
  const auto wait = model.step(delayed, {}, .01, memory);
  check(
    wait.valid && wait.prediction.phase == TransitionPhase::Braking &&
      wait.steering_targets == delayed.steering_angles,
    "zero commands do not replace measured stop evidence");
  delayed = {};
  delayed.actual_mode = DriveMode::Crab;
  memory = model.alignment_seed(delayed, target);
  for (int tick = 0; tick < 10; ++tick) {
    const auto hold = model.step(delayed, {}, .01, memory);  // measured joints deliberately lag
    check(
      hold.valid && hold.prediction.phase != TransitionPhase::Stable,
      "elapsed steering time cannot fake measured alignment");
    memory = hold.prediction;
    delayed.stamp_s += .01;
  }
  check(
    memory.alignment == target && memory.aligned_since_s < 0,
    "pending alignment target freezes and failed measurements reset dwell");
  memory.transition_start_s = -10;
  check(
    !model.step(delayed, {}, .01, memory).valid, "transition deadline rejects stalled alignment");
  state = moving(c, DriveMode::DualAckermann, {.3, 0, 0});
  memory = model.seed(state);
  const auto reverse = model.step(state, {-.4, 0, 0}, .1, memory);
  check(
    reverse.valid && !reverse.aligning && close(reverse.prediction.limited_velocity.vx, .21),
    "signed reversal uses the single current body intent slew law");
  memory.limited_velocity.vx = 0;  // command zero cleared slew while encoders coast
  const auto restart = model.step(state, {.4, 0, 0}, .01, memory);
  check(
    restart.valid && close(restart.prediction.limited_velocity.vx, .009),
    "command history is separate from encoder velocity");
}
void independent_pose_and_sweep()
{
  for (double dt : {.1, .2, .8}) {
    Config c;
    c.dt_s = dt;
    DriveModel model(c);
    for (Control u : {Control{.4, 0, .3}, {-.3, 0, -.2}}) {
      auto start = moving(c, DriveMode::DualAckermann, {.3, 0, .2});
      start.pose = {.2, -.1, .3};
      const auto whole = model.step(start, u, dt);
      check(whole.valid, "curved command prediction");
      auto nominal = start, independent = start;
      auto memory = model.seed(start);
      double excursion = 0;
      const double dx = whole.state.pose.x - start.pose.x, dy = whole.state.pose.y - start.pose.y;
      const int ticks = static_cast<int>(std::llround(dt / c.chassis_period_s));
      for (int tick = 0; tick < ticks; ++tick) {
        const auto next = model.step(nominal, u, c.chassis_period_s, memory);
        check(next.valid, "microstep prediction");
        test::PlantTargets target;
        target.samples.push_back(
          {next.steering_targets, next.wheel_speed_targets, c.chassis_period_s});
        test::actuate(independent, target, c);
        memory = next.prediction;
        nominal = next.state;
        const double length2 = dx * dx + dy * dy;
        const double fraction = length2 ? std::clamp(
                                            ((independent.pose.x - start.pose.x) * dx +
                                             (independent.pose.y - start.pose.y) * dy) /
                                              length2,
                                            0., 1.)
                                        : 0;
        excursion = std::max(
          excursion, std::hypot(
                       independent.pose.x - start.pose.x - fraction * dx,
                       independent.pose.y - start.pose.y - fraction * dy));
      }
      check(
        std::hypot(
          whole.state.pose.x - independent.pose.x, whole.state.pose.y - independent.pose.y) <
            1e-8 &&
          std::abs(angle_distance(whole.state.pose.yaw, independent.pose.yaw)) < 1e-9,
        "independent encoder integration agrees with held-target pose prediction");
      check(
        excursion <= whole.sweep_margin_m + 1e-9,
        "swept enclosure includes independent intermediate curved/reversal motion");
    }
  }
}
void transitions_and_sweep()
{
  Config c;
  for (DriveMode source : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
    for (DriveMode target : {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab}) {
      if (source == target) continue;
      auto s = moving(
        c, source,
        source == DriveMode::Spin   ? Control{0, 0, .3}
        : source == DriveMode::Crab ? Control{0, .3, 0}
                                    : Control{.3, 0, .2});
      std::size_t steps = 0;
      std::vector<Pose2d> poses;
      const double duration = TransitionModel(c).rollout(s, target, steps, 100, &poses);
      check(
        duration > 0 && steps == poses.size() && s.actual_mode == target && is_stopped(s, c),
        "all six nominal transitions include brake, measured alignment and confirmation allowance");
      s = {};
      steps = 0;
      check(
        TransitionModel(c).rollout(s, DriveMode::Spin, steps, 1) < 0,
        "bounded transition budget fails closed");
    }
  }
  c.dt_s = .8;
  c.robot_radius_m = .001;
  c.collision_margin_m = 0;
  auto state = moving(c, DriveMode::DualAckermann, {.03, 0, 0});
  const auto reversal = DriveModel(c).step(state, {-.04, 0, 0}, c.dt_s);
  check(
    reversal.valid && reversal.sweep_margin_m > 0,
    "signed reversal must enclose interior excursions");
  Trajectory t;
  t.valid = true;
  t.poses = {state.pose, reversal.state.pose};
  t.sweep_margins_m = {reversal.sweep_margin_m};
  ControllerInput in;
  in.vehicle = state;
  in.reference_path = {{0, 0, 0}};
  in.obstacles = {{.0005, 0, 0}};
  check(
    TrajectoryValidator(c).check(in, t) == TrajectoryStatus::Collision,
    "swept brake/drive reversal detects interior obstacle");
  c = {};
  check(!DriveModel(c).step({}, {.2, .1, 0}, .1).valid, "illegal mode velocity fails closed");
  check(
    !DriveModel(c).step({}, {.2, 0, 0}, std::numeric_limits<double>::infinity()).valid,
    "nonfinite period fails closed");
  check(!DriveModel(c).step({}, {.2, 0, 0}, 100).valid, "bounded microstep work");
}
}  // namespace
int main()
{
  try {
    kinematics_and_limits();
    zero_and_discrete_stop();
    history_and_alignment();
    independent_pose_and_sweep();
    transitions_and_sweep();
    std::cout << "Current chassis prediction regressions passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
