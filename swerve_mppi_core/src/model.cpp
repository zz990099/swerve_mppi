#include "swerve_mppi/model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace swerve_mppi
{
namespace chassis = swerve_chassis_core;
Model::Model(Config config) : config_(config), generator_(config.geometry, config.limits)
{
  validate(config_);
}
Velocity Model::project(Velocity v) const
{
  if (!chassis::finite(chassis::Twist{v.vx, v.vy, v.wz})) return {};
  double ratio = std::min(
    {1.0, config_.max_speed / std::max(1e-12, std::hypot(v.vx, v.vy)),
     config_.max_yaw_rate / std::max(1e-12, std::abs(v.wz))});
  // A zero translation cap still permits an in-place rotation.
  if (std::hypot(v.vx, v.vy) < 1e-12)
    ratio = std::min(1.0, config_.max_yaw_rate / std::max(1e-12, std::abs(v.wz)));
  auto t = chassis::scale({v.vx, v.vy, v.wz}, ratio);
  auto wheels = chassis::inverse(config_.geometry, t);
  ratio = 1;
  for (double speed : wheels.speed)
    ratio = std::min(ratio, config_.limits.wheel_speed / std::max(1e-12, std::abs(speed)));
  return {t.vx * ratio, t.vy * ratio, t.wz * ratio};
}
Velocity Model::condition(
  Velocity desired, Velocity previous, const std::array<double, 4> & reference, double dt) const
{
  if (
    !chassis::finite(chassis::Twist{desired.vx, desired.vy, desired.wz}) ||
    !chassis::finite(chassis::Twist{previous.vx, previous.vy, previous.wz}) || !std::isfinite(dt) ||
    dt <= 0 || dt > 1)
    throw std::invalid_argument("invalid request conditioning input");
  desired = project(desired);
  previous = project(previous);
  if (std::hypot(previous.vx, previous.vy) + .3 * std::abs(previous.wz) > .03) {
    const auto old =
      chassis::inverse(config_.geometry, {previous.vx, previous.vy, previous.wz}, reference);
    for (int iteration = 0; iteration < 12; ++iteration) {
      auto next =
        chassis::inverse(config_.geometry, {desired.vx, desired.vy, desired.wz}, old.steering);
      bool changed = false;
      for (std::size_t i = 0; i < 4; ++i) {
        if (
          std::abs(next.steering[i] - old.steering[i]) <= chassis::pi / 2 + 1e-6 ||
          std::abs(old.speed[i]) < .01)
          continue;
        double sign = old.speed[i] > 0 ? 1. : -1.;
        double y = (i % 2 == 0 ? 1. : -1.) * config_.geometry.track_width / 2;
        // Current rolling branch is a half-space in body twist. Projection onto
        // it retains lateral intent instead of requesting a near-180 degree flip.
        double value = sign * (desired.vx - desired.wz * y);
        if (value < 0) {
          const double radius =
            std::hypot(config_.geometry.wheelbase, config_.geometry.track_width) / 2;
          double denominator = 1 + y * y / (radius * radius);
          desired.vx -= value * sign / denominator;
          desired.wz += value * sign * y / (radius * radius * denominator);
          changed = true;
        }
      }
      if (!changed) break;
    }
    auto next =
      chassis::inverse(config_.geometry, {desired.vx, desired.vy, desired.wz}, old.steering);
    for (std::size_t i = 0; i < 4; ++i)
      if (std::abs(next.steering[i] - old.steering[i]) > chassis::pi / 2 + 1e-4) {
        desired = {};
        break;  // No continuous branch: brake before selecting a new intent.
      }
  }
  desired = project(desired);
  const double dv = std::hypot(desired.vx - previous.vx, desired.vy - previous.vy),
               dw = std::abs(desired.wz - previous.wz);
  double ratio = std::min(
    {1., config_.limits.linear_acceleration * dt / std::max(1e-12, dv),
     config_.limits.angular_acceleration * dt / std::max(1e-12, dw)});
  Velocity result{
    previous.vx + ratio * (desired.vx - previous.vx),
    previous.vy + ratio * (desired.vy - previous.vy),
    previous.wz + ratio * (desired.wz - previous.wz)};
  if (std::abs(result.vx) < 1e-8) result.vx = 0;
  if (std::abs(result.vy) < 1e-8) result.vy = 0;
  if (std::abs(result.wz) < 1e-8) result.wz = 0;
  return result;
}
bool Model::valid(const State & s) const
{
  if (
    !s.generator.valid || !std::isfinite(s.pose.x) || !std::isfinite(s.pose.y) ||
    !std::isfinite(s.pose.yaw) ||
    !chassis::finite(chassis::Twist{s.velocity.vx, s.velocity.vy, s.velocity.wz}) ||
    !chassis::finite(chassis::Twist{s.requested.vx, s.requested.vy, s.requested.wz}) ||
    !chassis::finite(chassis::Joints{s.steering, s.wheel_speeds}) ||
    !chassis::finite(s.generator.command))
    return false;
  for (std::size_t i = 0; i < 4; ++i) {
    if (
      std::abs(s.steering[i]) > config_.limits.steering + 0.03 ||
      std::abs(s.generator.command.steering[i]) > config_.limits.steering + 1e-9 ||
      std::abs(s.generator.command.speed[i]) > config_.limits.wheel_speed + 1e-9)
      return false;
  }
  return chassis::residual(config_.geometry, s.generator.command) < 1e-7 &&
         chassis::residual(config_.geometry, {s.steering, s.wheel_speeds}) <= config_.max_residual;
}
Trajectory Model::rollout(const State & initial, const std::vector<Velocity> & controls) const
{
  Trajectory out;
  out.controls = controls;
  out.states.reserve(controls.size() + 1);
  out.states.push_back(initial);
  if (!valid(initial)) {
    out.valid = false;
    return out;
  }
  State s = initial;
  int ticks = static_cast<int>(std::ceil(config_.model_dt / config_.integration_dt));
  const double dt = config_.model_dt / ticks;
  try {
    for (auto & u : out.controls) {
      if (!chassis::finite(chassis::Twist{u.vx, u.vy, u.wz})) {
        out.valid = false;
        return out;
      }
      u = project(u);
      for (int tick = 0; tick < ticks; ++tick) {
        const chassis::Joints measured{s.steering, s.wheel_speeds};
        auto result = generator_.step({u.vx, u.vy, u.wz}, measured, s.generator, dt);
        // A position servo does not instantly attain each rate-limited target.
        // Apply a first-order nominal response before the shared actuator limits.
        auto servo_target = result.state.command;
        const double gain = -std::expm1(-dt / config_.steering_time_constant);
        for (std::size_t i = 0; i < 4; ++i)
          servo_target.steering[i] =
            measured.steering[i] + gain * (servo_target.steering[i] - measured.steering[i]);
        auto next = chassis::response(measured, servo_target, config_.limits, dt);
        const double residual = chassis::residual(config_.geometry, next);
        if (!std::isfinite(residual) || residual > config_.max_residual) {
          out.valid = false;
          return out;
        }
        // Integrate measured nominal velocity before advancing actuator response.
        const auto body = chassis::forward(config_.geometry, measured);
        auto p = chassis::integrate({s.pose.x, s.pose.y, s.pose.yaw}, body, dt);
        s.pose = {p.x, p.y, p.yaw};
        for (std::size_t i = 0; i < 4; ++i) {
          out.actuator_cost +=
            dt *
            (std::pow((next.steering[i] - s.steering[i]) / (dt * config_.limits.steering_rate), 2) +
             std::pow(
               (next.speed[i] - s.wheel_speeds[i]) / (dt * config_.limits.wheel_acceleration), 2)) /
            4;
        }
        out.residual_cost += dt * std::pow(residual / config_.max_residual, 2);
        s.steering = next.steering;
        s.wheel_speeds = next.speed;
        s.generator = result.state;
        auto v = chassis::forward(config_.geometry, next);
        s.velocity = {v.vx, v.vy, v.wz};
      }
      s.requested = u;
      out.states.push_back(s);
    }
  } catch (const std::exception &) {
    out.valid = false;
  }
  return out;
}
}  // namespace swerve_mppi
