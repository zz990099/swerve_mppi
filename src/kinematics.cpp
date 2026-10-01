#include "swerve_mppi/model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace swerve_mppi {
namespace {
constexpr double kPi = 3.14159265358979323846;
}
double wrap_angle(double angle) { return std::remainder(angle, 2.0 * kPi); }
double angle_distance(double a, double b) { return wrap_angle(a - b); }
Kinematics::Kinematics(const Config &config) : config_(config) { validate(config_); }

WheelCommand Kinematics::inverse(const Control &u, const std::array<double, 4> &current) const {
  WheelCommand result;
  const double x[] = {config_.wheelbase_m / 2, config_.wheelbase_m / 2, -config_.wheelbase_m / 2,
                      -config_.wheelbase_m / 2};
  const double y[] = {config_.track_m / 2, -config_.track_m / 2, config_.track_m / 2,
                      -config_.track_m / 2};
  if (!std::isfinite(u.vx) || !std::isfinite(u.vy) || !std::isfinite(u.wz)) {
    result.valid = false;
    return result;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(current[i]) || std::abs(current[i]) > config_.steering_limit_rad) {
      result.valid = false;
      return result;
    }
    const double wx = u.vx - u.wz * y[i], wy = u.vy + u.wz * x[i];
    const double speed = std::hypot(wx, wy);
    result.angles[i] = current[i];
    if (speed < 1e-9)
      continue;
    const double angle = std::atan2(wy, wx);
    double distance = std::numeric_limits<double>::infinity();
    int selected_turn = 3;
    // Mechanical joints use direct distance: wrapping across a hard stop is forbidden.
    for (int k = -2; k <= 2; ++k) {
      const double candidate = angle + k * kPi;
      if (std::abs(candidate) > config_.steering_limit_rad + 1e-9)
        continue;
      const double bounded =
          std::clamp(candidate, -config_.steering_limit_rad, config_.steering_limit_rad);
      const double delta = std::abs(bounded - current[i]);
      if (delta < distance - 1e-9 ||
          (std::abs(delta - distance) <= 1e-9 && std::abs(k) < selected_turn)) {
        distance = delta;
        selected_turn = std::abs(k);
        result.angles[i] = bounded;
        result.speeds[i] = k % 2 == 0 ? speed : -speed;
      }
    }
    if (!std::isfinite(distance))
      result.valid = false;
  }
  return result;
}

Twist2d Kinematics::forward(const std::array<double, 4> &speeds,
                            const std::array<double, 4> &angles) const {
  const double x[] = {config_.wheelbase_m / 2, config_.wheelbase_m / 2, -config_.wheelbase_m / 2,
                      -config_.wheelbase_m / 2};
  const double y[] = {config_.track_m / 2, -config_.track_m / 2, config_.track_m / 2,
                      -config_.track_m / 2};
  Twist2d v;
  double moment = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    const double wx = speeds[i] * std::cos(angles[i]), wy = speeds[i] * std::sin(angles[i]);
    v.vx += wx / 4.0;
    v.vy += wy / 4.0;
    v.wz += -y[i] * wx + x[i] * wy;
    moment += x[i] * x[i] + y[i] * y[i];
  }
  v.wz /= moment;
  return v;
}
double Kinematics::max_module_residual(const std::array<double, 4> &speeds,
                                       const std::array<double, 4> &angles,
                                       const Twist2d &twist) const {
  if (!std::isfinite(twist.vx) || !std::isfinite(twist.vy) || !std::isfinite(twist.wz))
    return std::numeric_limits<double>::infinity();
  const double x[] = {config_.wheelbase_m / 2, config_.wheelbase_m / 2, -config_.wheelbase_m / 2,
                      -config_.wheelbase_m / 2};
  const double y[] = {config_.track_m / 2, -config_.track_m / 2, config_.track_m / 2,
                      -config_.track_m / 2};
  double maximum = 0;
  for (std::size_t i = 0; i < speeds.size(); ++i) {
    if (!std::isfinite(speeds[i]) || !std::isfinite(angles[i]))
      return std::numeric_limits<double>::infinity();
    maximum = std::max(maximum,
                       std::hypot(speeds[i] * std::cos(angles[i]) - (twist.vx - twist.wz * y[i]),
                                  speeds[i] * std::sin(angles[i]) - (twist.vy + twist.wz * x[i])));
  }
  return maximum;
}
} // namespace swerve_mppi
