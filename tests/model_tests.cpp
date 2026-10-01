#include "behavior_fixture.hpp"
#include "swerve_mppi/controller.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace swerve_mppi;
namespace {
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
bool close(double a, double b) { return std::abs(a - b) < 1e-8; }
void test_bounded_kinematics() {
  Config c;
  Kinematics k(c);
  const double pi = std::acos(-1.0);
  for (double angle : {-pi / 2, 0.0, pi / 2}) {
    std::array<double, 4> current;
    current.fill(angle);
    for (const Control &u : {Control{-.4, 0, 0}, Control{0, .4, 0}, Control{0, 0, .5},
                             Control{.4, 0, .3}, Control{-.3, .2, -.4}}) {
      const auto w = k.inverse(u, current);
      check(w.valid, "inverse kinematics must support all reachable directions");
      for (double target : w.angles)
        check(std::abs(target) <= c.steering_limit_rad, "steering must respect mechanical stops");
      const auto v = k.forward(w.speeds, w.angles);
      check(close(v.vx, u.vx) && close(v.vy, u.vy) && close(v.wz, u.wz),
            "bounded inverse/forward kinematics must round-trip signed wheel speeds");
    }
  }
  auto invalid = k.inverse({.2, 0, 0}, {pi, 0, 0, 0});
  check(!invalid.valid, "out-of-range measured joints must be rejected");
  DriveModel model(c);
  for (double angle : model.steering_for_mode(DriveMode::Spin))
    check(std::abs(angle) <= pi / 2, "spin mode request must respect +/-90 degree stops");
}
void test_braking_and_wheel_consistency() {
  Config c;
  c.max_linear_accel_mps2 = .9;
  c.max_linear_decel_mps2 = .2;
  DriveModel model(c);
  Kinematics k(c);
  VehicleState s;
  s.velocity.vx = .5;
  s.wheel_speeds.fill(.5);
  const auto braking = model.step(s, {}, .1);
  check(braking.valid && close(braking.state.velocity.vx, .48),
        "braking must use the configured deceleration");
  const auto v = k.forward(braking.wheel_speed_targets, braking.state.steering_angles);
  check(close(v.vx, braking.state.velocity.vx) && close(v.wz, braking.state.velocity.wz),
        "predicted body and actuator speeds must agree");
  s = {};
  Control u{.5, 0, .4};
  s.steering_angles = k.inverse(u, {}).angles;
  const auto accelerating = model.step(s, u, .1);
  const auto measured =
      k.forward(accelerating.wheel_speed_targets, accelerating.state.steering_angles);
  check(close(measured.vx, accelerating.state.velocity.vx) &&
            close(measured.wz, accelerating.state.velocity.wz),
        "turn acceleration must preserve wheel/body consistency");
  check(std::hypot(measured.vx, measured.vy) <= c.max_linear_accel_mps2 * .1 + 1e-9,
        "body linear acceleration must remain bounded");
}
// Independent signed-speed time budget: crossing zero requires braking first.
double minimum_speed_change_time(double before, double after, double accel, double decel) {
  if (before * after < 0)
    return std::abs(before) / decel + std::abs(after) / accel;
  return std::abs(after - before) / (std::abs(after) < std::abs(before) ? decel : accel);
}
void test_reversal_time_budget() {
  for (double decel : {.1, .9}) {
    Config c;
    c.max_linear_decel_mps2 = c.max_angular_decel_radps2 = decel;
    c.max_linear_accel_mps2 = c.max_angular_accel_radps2 = decel == .1 ? .9 : .1;
    DriveModel model(c);
    Kinematics k(c);
    for (auto mode : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
      for (double direction : {-1.0, 1.0}) {
        for (double speed : {.03, .3}) {
          for (double target : {.04, .4}) {
            for (double dt : {.05, .1, .2}) {
              VehicleState state;
              state.actual_mode = mode;
              Control initial, intent;
              if (mode == DriveMode::Spin) {
                initial.wz = direction * speed;
                intent.wz = -direction * target;
              } else if (mode == DriveMode::Crab) {
                initial.vy = direction * speed;
                intent.vy = -direction * target;
              } else {
                initial.vx = direction * speed;
                intent.vx = -direction * target;
              }
              const auto wheels = k.inverse(initial, {});
              state.wheel_speeds = wheels.speeds;
              state.steering_angles = wheels.angles;
              state.velocity = {initial.vx, initial.vy, initial.wz};
              const auto next = model.step(state, intent, dt);
              check(next.valid && !next.aligning, "signed reversal must remain a drive step");
              const double after = mode == DriveMode::Spin   ? next.state.velocity.wz
                                   : mode == DriveMode::Crab ? next.state.velocity.vy
                                                             : next.state.velocity.vx;
              check(minimum_speed_change_time(direction * speed, after, c.max_linear_accel_mps2,
                                              decel) <= dt + 1e-8,
                    "reversal cannot spend less than the independent braking/acceleration budget");
            }
          }
        }
      }
    }
  }
  Config c;
  c.max_linear_decel_mps2 = .1;
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.time_in_mode_s = 2;
  in.vehicle.velocity.vx = .03;
  in.vehicle.wheel_speeds.fill(.03);
  in.reference_path = {{0, 0, 0}, {-.2, 0, 0}};
  const auto out = Controller(c).compute(in);
  check(out.action == Action::Drive && out.body_command.vx >= .02 - 1e-8,
        "reverse capture must brake within the configured time budget before reversing");
}
void test_analytic_stopping_motion() {
  // Independent closed form: proportional braking has distance v*T/2 and
  // yaw w*T/2, with T set by the slowest body or rolling-speed limit.
  for (double dt : {.05, .1, .2}) {
    for (double speed : {.003, .1, .35}) {
      for (double sign : {-1.0, 1.0}) {
        Config c;
        c.dt_s = dt;
        c.horizon_steps = 2;
        c.max_linear_decel_mps2 = .2;
        c.max_wheel_accel_mps2 = .5;
        for (auto mode : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
          VehicleState s;
          s.actual_mode = mode;
          const Control initial = mode == DriveMode::Spin   ? Control{0, 0, sign * speed}
                                  : mode == DriveMode::Crab ? Control{0, sign * speed, 0}
                                                            : Control{sign * speed, 0, 0};
          const auto joint = Kinematics(c).inverse(initial, {});
          s.steering_angles = joint.angles;
          s.wheel_speeds = joint.speeds;
          s.velocity = {initial.vx, initial.vy, initial.wz};
          double duration = std::max(std::hypot(initial.vx, initial.vy) / c.max_linear_decel_mps2,
                                     std::abs(initial.wz) / c.max_angular_decel_radps2);
          for (double wheel : joint.speeds)
            duration = std::max(duration, std::abs(wheel) / c.max_wheel_accel_mps2);
          Trajectory stop;
          RolloutEngine(c).generate_stop(s, stop);
          check(stop.valid && close(stop.final_state.pose.x, initial.vx * duration / 2) &&
                    close(stop.final_state.pose.y, initial.vy * duration / 2) &&
                    close(stop.final_state.pose.yaw, initial.wz * duration / 2),
                "complete braking must match the independent continuous stopping oracle");
          check(stop.final_state.wheel_speeds == std::array<double, 4>{},
                "a stopping threshold cannot truncate residual sub-threshold displacement");
        }
      }
    }
  }
  Config c;
  c.collision_margin_m = 0;
  ControllerInput in;
  in.vehicle.stamp_s = 1;
  in.vehicle.velocity.vx = .1;
  in.vehicle.wheel_speeds.fill(.1);
  in.reference_path = {{0, 0, 0}};
  in.obstacles = {{.553, 0, .05}};
  Trajectory stop;
  RolloutEngine(c).generate_stop(in.vehicle, stop);
  check(close(stop.final_state.pose.x, .005) &&
            TrajectoryValidator(c).check(in, stop) == TrajectoryStatus::Collision &&
            Controller(c).compute(in).action == Action::SafeStop,
        "an obstacle inside the analytic braking distance must reject the stop");
}
void test_curved_stop_and_reversal_enclosures() {
  Config c;
  c.dt_s = .2;
  VehicleState start;
  const Control initial{.4, 0, .3};
  const auto joint = Kinematics(c).inverse(initial, {});
  start.steering_angles = joint.angles;
  start.wheel_speeds = joint.speeds;
  start.velocity = {initial.vx, initial.vy, initial.wz};
  double duration = std::max(.4 / c.max_linear_decel_mps2, .3 / c.max_angular_decel_radps2);
  for (double wheel : joint.speeds)
    duration = std::max(duration, std::abs(wheel) / c.max_wheel_accel_mps2);
  Trajectory stop;
  RolloutEngine(c).generate_stop(start, stop);
  const double angle = .3 * duration / 2, radius = .4 / .3;
  check(stop.valid &&
            std::hypot(stop.final_state.pose.x - radius * std::sin(angle),
                       stop.final_state.pose.y - radius * (1 - std::cos(angle))) <=
                stop.position_error_m + 1e-10 &&
            close(stop.final_state.pose.yaw, angle),
        "curved braking must enclose the independent constant-curvature integral");
  check(stop.sweep_margins_m.size() + 1 == stop.poses.size(),
        "every curved motion segment must carry its conservative sweep enclosure");

  c.dt_s = .4;
  c.robot_radius_m = .001;
  c.collision_margin_m = 0;
  c.max_linear_decel_mps2 = .1;
  c.max_linear_accel_mps2 = .9;
  start = {};
  start.velocity.vx = .03;
  start.wheel_speeds.fill(.03);
  const auto reversal = DriveModel(c).step(start, {-.04, 0, 0}, c.dt_s);
  const double brake_time = .03 / .1, accel_time = .04 / .9;
  const double expected =
      .03 * brake_time / 2 - .04 * accel_time / 2 - .04 * (c.dt_s - brake_time - accel_time);
  check(reversal.valid && close(reversal.state.pose.x, expected),
        "reversal displacement must include braking, acceleration, and final-speed hold");
  Trajectory trace;
  trace.valid = true;
  trace.poses = {start.pose, reversal.state.pose};
  trace.sweep_margins_m = {reversal.sweep_margin_m};
  ControllerInput in;
  in.vehicle = start;
  in.reference_path = {{0, 0, 0}};
  in.obstacles = {{.0035, 0, 0}};
  check(TrajectoryValidator(c).check(in, trace) == TrajectoryStatus::Collision,
        "the sweep must detect a reversal excursion outside its endpoint chord");
  trace.sweep_margins_m.clear();
  check(TrajectoryValidator(c).check(in, trace) == TrajectoryStatus::Valid,
        "the independent reversal example must actually be missed by the endpoint chord");
  trace.sweep_margins_m = {-1};
  check(TrajectoryValidator(c).check(in, trace) == TrajectoryStatus::Invalid,
        "invalid swept-motion metadata must fail closed");
}
void test_independent_ramp_integration_and_fixture() {
  for (double dt : {.05, .1, .2}) {
    Config c;
    c.dt_s = dt;
    VehicleState start;
    start.pose.yaw = .3;
    const Control initial{.3, 0, .2};
    const auto joint = Kinematics(c).inverse(initial, {});
    start.steering_angles = joint.angles;
    start.wheel_speeds = joint.speeds;
    start.velocity = {initial.vx, initial.vy, initial.wz};
    const auto step = DriveModel(c).step(start, {.4, 0, .3}, dt);
    check(step.valid && !step.aligning, "independent ramp oracle must use a moving Drive");
    const auto &end = step.state.velocity;
    // This example only increases linear/angular speed: derive its ramp time
    // directly from the endpoint changes, without using the production profile.
    check(end.vx >= initial.vx && end.wz >= initial.wz,
          "oracle assumptions require increasing speed and angular rate");
    double duration = std::max(std::hypot(end.vx - initial.vx, end.vy) / c.max_linear_accel_mps2,
                               std::abs(end.wz - initial.wz) / c.max_angular_accel_radps2);
    for (std::size_t j = 0; j < 4; ++j)
      duration = std::max(
          {duration,
           std::abs(step.state.wheel_speeds[j] - start.wheel_speeds[j]) / c.max_wheel_accel_mps2,
           std::abs(step.state.steering_angles[j] - start.steering_angles[j]) /
               c.max_steer_rate_radps});
    Pose2d oracle = start.pose;
    constexpr int substeps = 20000;
    const double h = dt / substeps;
    for (int i = 0; i < substeps; ++i) {
      const double time = (i + .5) * h;
      const double f = std::min(1.0, time / duration);
      const double vx = initial.vx + f * (end.vx - initial.vx), vy = f * end.vy;
      const double wz = initial.wz + f * (end.wz - initial.wz);
      const double heading = oracle.yaw + wz * h / 2;
      oracle.x += (std::cos(heading) * vx - std::sin(heading) * vy) * h;
      oracle.y += (std::sin(heading) * vx + std::cos(heading) * vy) * h;
      oracle.yaw += wz * h;
    }
    check(std::hypot(oracle.x - step.state.pose.x, oracle.y - step.state.pose.y) <=
                  step.integration_error_m + 1e-9 &&
              std::abs(oracle.yaw - step.state.pose.yaw) < 1e-8,
          "the quadrature enclosure must contain an independent fine-step body ramp");
  }
  Config c;
  VehicleState measured;
  measured.velocity.vx = .1;
  measured.wheel_speeds.fill(.1);
  ExecutionResult brake;
  brake.action = Action::Brake;
  brake.feedback.confirmed = true;
  test::actuate(measured, brake, c);
  check(close(measured.pose.x, .005) && measured.wheel_speeds == std::array<double, 4>{},
        "the independent encoder fixture must include continuous braking displacement");
}
void test_braking_before_steering() {
  Config c;
  DriveModel model(c);
  VehicleState s;
  s.actual_mode = DriveMode::Crab;
  s.velocity.vx = .4;
  s.wheel_speeds.fill(.4);
  auto next = model.step(s, {0, .4, 0}, .1);
  check(next.valid && next.state.steering_angles == s.steering_angles,
        "moving wheels must brake before steering realignment");
  check(next.steering_targets == s.steering_angles,
        "braking action must retain measured steering targets");
  for (int i = 0; i < 30; ++i)
    s = model.step(s, {0, .4, 0}, .1).state;
  check(s.pose.y > 0 && std::abs(s.pose.yaw) < 1e-8,
        "stop/align/drive sequence must eventually make lateral progress");
}
void test_continuous_steering_limits() {
  Config c;
  c.max_steer_rate_radps = .3;
  c.max_angular_accel_radps2 = .15;
  c.max_angular_decel_radps2 = .12;
  c.max_linear_accel_mps2 = .3;
  c.max_linear_decel_mps2 = .2;
  DriveModel model(c);
  Kinematics k(c);
  VehicleState state;
  state.wheel_speeds.fill(.4);
  state.velocity.vx = .4;
  for (int tick = 0; tick < 80; ++tick) {
    const Control control{.4, 0, .12 * std::sin(tick * .04)};
    const auto next = model.step(state, control, c.dt_s);
    check(next.valid && !next.aligning, "smooth curvature changes must drive without stopping");
    for (std::size_t i = 0; i < 4; ++i) {
      check(std::abs(next.state.steering_angles[i] - state.steering_angles[i]) <=
                c.max_steer_rate_radps * c.dt_s + 1e-9,
            "moving steering rate exceeded");
      check(std::abs(next.state.wheel_speeds[i] - state.wheel_speeds[i]) <=
                c.max_wheel_accel_mps2 * c.dt_s + 1e-9,
            "moving wheel acceleration exceeded");
    }
    const auto v = next.state.velocity;
    const auto before = state.velocity;
    const double linear = std::hypot(v.vx, v.vy) < std::hypot(before.vx, before.vy)
                              ? c.max_linear_decel_mps2
                              : c.max_linear_accel_mps2;
    const double angular = std::abs(v.wz) < std::abs(before.wz) ? c.max_angular_decel_radps2
                                                                : c.max_angular_accel_radps2;
    check(std::hypot(v.vx - before.vx, v.vy - before.vy) <= linear * c.dt_s + 1e-9 &&
              std::abs(v.wz - before.wz) <= angular * c.dt_s + 1e-9,
          "steering and wheel acceleration must be limited together");
    const auto implied = k.forward(next.wheel_speed_targets, next.steering_targets);
    check(close(implied.vx, v.vx) && close(implied.vy, v.vy) && close(implied.wz, v.wz),
          "drive body command must describe the complete joint target step");
    state = next.state;
  }
  check(state.pose.x > 2, "continuous steering must sustain forward progress");
  state = {};
  state.actual_mode = DriveMode::Crab;
  state.steering_angles.fill(c.steering_limit_rad);
  state.wheel_speeds.fill(.4);
  state.velocity.vy = .4;
  const auto crossing = model.step(state, {-.01, .4, 0}, c.dt_s);
  check(crossing.aligning && crossing.steering_targets == state.steering_angles,
        "an equivalent wheel direction across the hard stop must still brake first");
}
void test_rate_limits_and_projection() {
  Config c;
  c.max_wheel_accel_mps2 = .1;
  c.max_wheel_speed_mps = .2;
  DriveModel model(c);
  const auto projected = model.project({.8, 0, .7}, DriveMode::DualAckermann);
  auto s = VehicleState{};
  s.steering_angles = Kinematics(c).inverse(projected, {}).angles;
  const auto step = model.step(s, projected, .1);
  check(step.valid, "wheel-speed projection must make a feasible control");
  for (double speed : step.state.wheel_speeds)
    check(std::abs(speed) <= .01 + 1e-9, "wheel acceleration must be limited");
}
void test_zero_delay_transition_and_confirmation() {
  Config c;
  c.alignment_min_s = 0;
  c.confirmation_prediction_s = 0;
  c.minimum_mode_dwell_s = 0;
  TransitionModel transition(c);
  VehicleState s;
  std::size_t steps = 0;
  std::vector<Pose2d> trace;
  check(transition.rollout(s, DriveMode::Crab, steps, 10, &trace) > 0 && steps == trace.size(),
        "zero-delay transitions must advance time and terminate");
  ModeManager manager(c);
  s.stamp_s = 1;
  manager.begin(DriveMode::Spin, {}, s);
  s.velocity = {};
  s.wheel_speeds.fill(.1);
  check(manager.update(s).action == Action::Brake,
        "zero odometry with spinning wheels must not confirm a stopped chassis");
}
void test_configuration_ownership() {
  DriveModel model(Config{});
  ModeManager manager(Config{});
  check(model.step(VehicleState{}, {.2, 0, 0}, .1).valid,
        "models must own configuration passed as a temporary");
  VehicleState s;
  s.stamp_s = 1;
  manager.begin(DriveMode::Crab, {}, s);
  check(manager.update(s).action == Action::RequestMode,
        "manager temporary configuration must stay valid");
  Config invalid;
  invalid.steering_limit_rad = 1;
  bool rejected = false;
  try {
    validate(invalid);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "unsupported steering travel must be rejected");
}
} // namespace
int main() {
  try {
    test_bounded_kinematics();
    test_braking_and_wheel_consistency();
    test_reversal_time_budget();
    test_analytic_stopping_motion();
    test_curved_stop_and_reversal_enclosures();
    test_independent_ramp_integration_and_fixture();
    test_braking_before_steering();
    test_continuous_steering_limits();
    test_rate_limits_and_projection();
    test_zero_delay_transition_and_confirmation();
    test_configuration_ownership();
    std::cout << "Model regressions passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
