#include "swerve_mppi/model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace swerve_mppi
{
Model::Model(Config config) : config_(config) { validate(config_); }
Velocity Model::project(Velocity v, Mode mode) const
{
  if (mode == Mode::Spin) {
    v = {0, 0, std::clamp(v.wz, -config_.max_yaw_rate, config_.max_yaw_rate)};
  } else if (mode == Mode::Ackermann) {
    v.vy = 0;
    v.vx = std::clamp(v.vx, -config_.max_speed, config_.max_speed);
    double w = std::min(config_.max_yaw_rate, std::abs(v.vx) / config_.min_turn_radius);
    v.wz = std::clamp(v.wz, -w, w);
  } else {
    v.wz = 0;
    double scale = std::max(1.0, std::hypot(v.vx, v.vy) / config_.max_speed);
    v.vx /= scale;
    v.vy /= scale;
  }
  const auto wheels = inverse(v, {});
  double scale = 1;
  for (double s : wheels.speeds) scale = std::max(scale, std::abs(s) / config_.max_wheel_speed);
  return {v.vx / scale, v.vy / scale, v.wz / scale};
}
Wheels Model::inverse(Velocity v, const std::array<double, 4> & current) const
{
  Wheels out;
  const double pi = std::acos(-1.0);
  for (std::size_t i = 0; i < 4; ++i) {
    double x = (i < 2 ? 1 : -1) * config_.wheelbase / 2,
           y = (i % 2 == 0 ? 1 : -1) * config_.track / 2;
    double wx = v.vx - v.wz * y, wy = v.vy + v.wz * x, speed = std::hypot(wx, wy);
    out.angles[i] = current[i];
    if (speed < 1e-12) continue;
    double best = std::numeric_limits<double>::infinity();
    for (int k = -1; k <= 1; ++k) {
      double a = std::atan2(wy, wx) + k * pi;
      if (std::abs(a) > config_.steer_limit + 1e-9) continue;
      if (std::abs(a - current[i]) < best) {
        best = std::abs(a - current[i]);
        out.angles[i] = std::clamp(a, -config_.steer_limit, config_.steer_limit);
        out.speeds[i] = (k % 2 == 0 ? speed : -speed);
      }
    }
    if (!std::isfinite(best)) out.speeds[i] = std::numeric_limits<double>::quiet_NaN();
  }
  return out;
}
Velocity Model::forward(const Wheels & wheels) const
{
  Velocity v;
  double moment = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    double x = (i < 2 ? 1 : -1) * config_.wheelbase / 2,
           y = (i % 2 == 0 ? 1 : -1) * config_.track / 2;
    double wx = wheels.speeds[i] * std::cos(wheels.angles[i]),
           wy = wheels.speeds[i] * std::sin(wheels.angles[i]);
    v.vx += wx / 4;
    v.vy += wy / 4;
    v.wz += x * wy - y * wx;
    moment += x * x + y * y;
  }
  v.wz /= moment;
  return v;
}
Trajectory Model::rollout(
  const State & initial, Mode target, const std::vector<Velocity> & controls) const
{
  Trajectory out;
  out.mode = target;
  out.controls = controls;
  out.states.reserve(controls.size() + 1);
  State s = initial;
  out.states.push_back(s);
  Velocity limited = s.velocity;
  bool aligning = target != s.mode;
  std::array<double, 4> alignment{};
  double dwell = 0;
  if (aligning && !controls.empty()) {
    auto entry = controls.front();
    if (entry.vx == 0 && entry.vy == 0 && entry.wz == 0)
      entry = target == Mode::Spin ? Velocity{0, 0, 1} : Velocity{1, 0, 0};
    alignment = inverse(entry, s.steering).angles;
  }
  const int ticks = static_cast<int>(std::ceil(config_.model_dt / config_.integration_dt));
  const double h = config_.model_dt / ticks;
  auto approach = [](double a, double b, double d) { return a + std::clamp(b - a, -d, d); };
  for (const auto & requested : controls) {
    const auto v = project(requested, target);
    for (int tick = 0; tick < ticks; ++tick) {
      // Measured/nominal body motion is integrated before updating actuator response.
      double yaw = s.pose.yaw + s.velocity.wz * h / 2;
      s.pose.x += (std::cos(yaw) * s.velocity.vx - std::sin(yaw) * s.velocity.vy) * h;
      s.pose.y += (std::sin(yaw) * s.velocity.vx + std::cos(yaw) * s.velocity.vy) * h;
      s.pose.yaw = wrap_angle(s.pose.yaw + s.velocity.wz * h);
      Wheels desired;
      if (!aligning) {
        if (v.vx == 0 && v.vy == 0 && v.wz == 0) {
          limited = {};
          desired.angles = s.steering;
        } else {
          double dx = v.vx - limited.vx, dy = v.vy - limited.vy, dw = v.wz - limited.wz;
          double ratio = std::max(
            {1.0, std::hypot(dx, dy) / (config_.linear_accel * h),
             std::abs(dw) / (config_.angular_accel * h)});
          limited = {limited.vx + dx / ratio, limited.vy + dy / ratio, limited.wz + dw / ratio};
          desired = inverse(limited, s.steering);
          for (std::size_t i = 0; i < 4; ++i)
            if (std::abs(desired.angles[i] - s.steering[i]) > config_.realign_angle)
              aligning = true;
          if (aligning) {
            alignment = inverse(v, s.steering).angles;
            dwell = 0;
          }
        }
      }
      if (aligning) {
        limited = {};
        bool stopped = true;
        for (std::size_t i = 0; i < 4; ++i) {
          s.wheel_speeds[i] = approach(s.wheel_speeds[i], 0, config_.wheel_accel * h);
          stopped = stopped && std::abs(s.wheel_speeds[i]) <= config_.stopped_wheel_speed;
        }
        if (stopped) {
          bool aligned = true;
          for (std::size_t i = 0; i < 4; ++i) {
            s.steering[i] = approach(s.steering[i], alignment[i], config_.steer_rate * h);
            aligned =
              aligned && std::abs(s.steering[i] - alignment[i]) <= config_.alignment_tolerance;
          }
          dwell = aligned ? dwell + h : 0;
          if (dwell >= config_.alignment_dwell) {
            aligning = false;
            s.mode = target;
          }
        }
      } else {
        for (std::size_t i = 0; i < 4; ++i) {
          s.wheel_speeds[i] =
            approach(s.wheel_speeds[i], desired.speeds[i], config_.wheel_accel * h);
          s.steering[i] = approach(s.steering[i], desired.angles[i], config_.steer_rate * h);
        }
      }
      s.velocity = forward({s.wheel_speeds, s.steering});
      s.switching = aligning;
      s.requested_mode = target;
      if (!std::isfinite(s.pose.x) || !std::isfinite(s.pose.y) || !std::isfinite(s.velocity.vx)) {
        out.valid = false;
        return out;
      }
    }
    out.states.push_back(s);
  }
  return out;
}
}  // namespace swerve_mppi
