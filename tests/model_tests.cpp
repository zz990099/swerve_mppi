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
