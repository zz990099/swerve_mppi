#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <swerve_chassis_core/kinematics.hpp>
#include <vector>

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/path.hpp"

namespace swerve_mppi
{
// Adaptive time parameterization of an ordered geometric path. Body heading
// comes from the path, independently of its direction of travel. Wheel targets
// below are per unit arc speed, hence include the yaw/translation coupling.
class ReferenceProfile
{
public:
  ReferenceProfile(const std::vector<Pose> & points, const Config & config)
  : points_(points), path_(points_), config_(config)
  {
    validate(config_);
    if (points.empty() || points.size() > 4096 || !std::isfinite(path_.length()))
      throw std::invalid_argument("invalid reference path");
    for (std::size_t i = 1; i + 1 < points.size(); ++i)
      if (path_.turn(i) >= PathView::stop_angle) corners_.push_back(path_.vertex_arc(i));
    auto count = static_cast<std::size_t>(std::clamp(std::ceil(path_.length() / .025), 1., 512.));
    s_.resize(count + 1);
    for (std::size_t i = 0; i <= count; ++i) s_[i] = path_.length() * i / count;
    s_.insert(s_.end(), corners_.begin(), corners_.end());
    std::sort(s_.begin(), s_.end());
    s_.erase(
      std::unique(s_.begin(), s_.end(), [](double a, double b) { return std::abs(a - b) < 1e-9; }),
      s_.end());
    count = s_.size() - 1;
    speed_.resize(count + 1);
    std::vector<swerve_chassis_core::Joints> wheels(count + 1);
    std::array<double, 4> previous{};
    for (std::size_t i = 0; i <= count; ++i) {
      const double s = s_[i];
      const auto u = unit(s, path_.at(s).yaw);
      wheels[i] = swerve_chassis_core::inverse(config_.geometry, {u.vx, u.vy, u.wz}, previous);
      previous = wheels[i].steering;
      double cap = path_.speed_limit(
        s, config_.max_speed, config_.limits.linear_acceleration, config_.approach_gain,
        config_.limits.steering_rate);
      cap = std::min(cap, config_.max_yaw_rate / std::max(1e-9, std::abs(u.wz)));
      for (double v : wheels[i].speed)
        cap = std::min(cap, config_.limits.wheel_speed / std::max(1e-9, std::abs(v)));
      for (double corner : corners_)
        if (std::abs(s - corner) < 1e-8) cap = 0;
      speed_[i] = cap;
    }
    // Both target rate and expected servo tracking error constrain arc speed.
    const double steering_rate = std::min(
      .8 * config_.limits.steering_rate,
      .8 * config_.limits.tracking_tolerance / config_.steering_time_constant);
    for (std::size_t i = 1; i <= count; ++i) {
      const double ds = s_[i] - s_[i - 1];
      if (ds < 1e-9) continue;
      // Mandatory geometric corners are stop/reorientation events, not a
      // steering derivative to be spread into the incoming straight segment.
      if (std::any_of(corners_.begin(), corners_.end(), [&](double corner) {
            return corner > s_[i - 1] + 1e-8 && corner <= s_[i] + 1e-8;
          }))
        continue;
      double cap = config_.max_speed;
      for (std::size_t w = 0; w < 4; ++w) {
        const double delta = std::abs(wheels[i].steering[w] - wheels[i - 1].steering[w]);
        cap = std::min(
          cap,
          delta > swerve_chassis_core::pi / 2 ? 0. : steering_rate * ds / std::max(1e-9, delta));
        const double dq = std::abs(wheels[i].speed[w] - wheels[i - 1].speed[w]) / ds;
        // q(s)*s_dot is wheel angular speed. This bounds its geometric
        // acceleration term q'(s)*s_dot^2; request slew also bounds s_ddot.
        cap = std::min(cap, std::sqrt(.5 * config_.limits.wheel_acceleration / std::max(1e-9, dq)));
      }
      speed_[i] = std::min(speed_[i], cap);
      speed_[i - 1] = std::min(speed_[i - 1], cap);
    }
    for (std::size_t i = count; i > 0; --i)
      speed_[i - 1] = std::min(
        speed_[i - 1], std::sqrt(
                         speed_[i] * speed_[i] +
                         2 * .7 * config_.limits.linear_acceleration * (s_[i] - s_[i - 1])));
    for (std::size_t i = 1; i <= count; ++i)
      speed_[i] = std::min(
        speed_[i], std::sqrt(
                     speed_[i - 1] * speed_[i - 1] +
                     2 * .7 * config_.limits.linear_acceleration * (s_[i] - s_[i - 1])));
  }
  ReferenceProfile(const ReferenceProfile &) = delete;
  ReferenceProfile & operator=(const ReferenceProfile &) = delete;
  double speed(double arc) const
  {
    if (path_.length() < 1e-8) return 0;
    auto it = std::upper_bound(s_.begin(), s_.end(), arc);
    const auto i = static_cast<std::size_t>(it - s_.begin());
    if (!i) return speed_.front();
    if (i >= s_.size()) return speed_.back();
    const double t = (arc - s_[i - 1]) / (s_[i] - s_[i - 1]);
    return speed_[i - 1] + t * (speed_[i] - speed_[i - 1]);
  }
  Velocity unit(double arc, double body_yaw) const
  {
    if (path_.length() < 1e-8) return {};
    double a = std::max(0., arc - .03), b = std::min(path_.length(), arc + .03);
    for (double corner : corners_) {
      if (corner > arc + 1e-8)
        b = std::min(b, corner);
      else
        a = std::max(a, corner);
    }
    const auto pa = path_.at(a), pb = path_.at(b);
    const double length = std::max(1e-9, distance(pa, pb));
    const double x = (pb.x - pa.x) / length, y = (pb.y - pa.y) / length;
    return {
      std::cos(body_yaw) * x + std::sin(body_yaw) * y,
      -std::sin(body_yaw) * x + std::cos(body_yaw) * y, path_.yaw_derivative(arc)};
  }

private:
  std::vector<Pose> points_;
  PathView path_;
  Config config_;
  std::vector<double> s_, speed_, corners_;
};
}  // namespace swerve_mppi
