#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <swerve_chassis_core/kinematics.hpp>
#include <vector>

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/heading_reference.hpp"
#include "swerve_mppi/path.hpp"

namespace swerve_mppi
{
// Nominal XY speed profile. Ordinary tracking evaluates its envelope at the
// current body heading without prescribing a yaw derivative. Explicit internal
// references retain their yaw/translation coupling. Every candidate is still
// checked using its own predicted actuator/body state.
class ReferenceProfile
{
public:
  ReferenceProfile(
    const std::vector<Pose> & points, const Config & config, std::optional<double> body_yaw = {})
  : points_(points), path_(points_), config_(config), body_yaw_(body_yaw)
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
    reasons_.assign(count + 1, 1);
    std::vector<swerve_chassis_core::Joints> wheels(count + 1);
    std::array<double, 4> previous{};
    for (std::size_t i = 0; i <= count; ++i) {
      const double s = s_[i];
      const auto u = unit(s, body_yaw_.value_or(path_.at(s).yaw));
      wheels[i] = swerve_chassis_core::inverse(config_.geometry, {u.vx, u.vy, u.wz}, previous);
      previous = wheels[i].steering;
      double cap = path_.speed_limit(
        s, config_.max_speed, config_.limits.linear_acceleration, config_.approach_gain,
        config_.limits.steering_rate);
      const double yaw_cap = config_.max_yaw_rate / std::max(1e-9, std::abs(u.wz));
      if (yaw_cap < cap) {
        cap = yaw_cap;
        reasons_[i] = 2;
      }
      for (double v : wheels[i].speed) {
        const double wheel_cap = config_.limits.wheel_speed / std::max(1e-9, std::abs(v));
        if (wheel_cap < cap) {
          cap = wheel_cap;
          reasons_[i] = 4;
        }
      }
      for (double corner : corners_)
        if (std::abs(s - corner) < 1e-8) {
          cap = 0;
          reasons_[i] = 8;
        }
      speed_[i] = cap;
    }
    // Common route-only preference: no candidate can lower its own weight by
    // slowing, stopping or turning more. Preview is bounded by the supplied
    // ordered section; direction changes beyond its end are not invented.
    std::vector<double> turns(count + 1, 0);
    for (std::size_t i = 1; i <= count; ++i)
      turns[i] = turns[i - 1] + std::abs(wrap_angle(path_.at(s_[i]).yaw - path_.at(s_[i - 1]).yaw));
    preference_.resize(count + 1);
    for (std::size_t i = 0; i <= count; ++i) {
      const double end = std::min(path_.length(), s_[i] + config_.heading_preference_distance);
      auto it = std::lower_bound(s_.begin(), s_.end(), end);
      const auto j = static_cast<std::size_t>(it - s_.begin());
      const double change = turns[std::min(j, count)] - turns[i];
      preference_[i] = (end - s_[i]) / config_.heading_preference_distance /
                       (1 + std::pow(change / config_.heading_preference_turn, 2));
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
        // A representation change requires braking and measured reorientation,
        // handled by condition()/Generator and the controller's latched intent.
        // A permanent zero-speed interval here has no restart condition.
        cap = std::min(
          cap, delta > swerve_chassis_core::pi / 2 ? config_.max_speed
                                                   : steering_rate * ds / std::max(1e-9, delta));
        if (
          delta > swerve_chassis_core::pi / 2 &&
          (branches_.empty() || std::abs(branches_.back() - s_[i - 1]) > 1e-8))
          branches_.push_back(s_[i - 1]);
        const double dq = std::abs(wheels[i].speed[w] - wheels[i - 1].speed[w]) / ds;
        // q(s)*s_dot is wheel angular speed. This bounds its geometric
        // acceleration term q'(s)*s_dot^2; request slew also bounds s_ddot.
        cap = std::min(cap, std::sqrt(.5 * config_.limits.wheel_acceleration / std::max(1e-9, dq)));
      }
      if (cap < speed_[i]) {
        speed_[i] = cap;
        reasons_[i] = 16;
      }
      if (cap < speed_[i - 1]) {
        speed_[i - 1] = cap;
        reasons_[i - 1] = 16;
      }
    }
    for (std::size_t i = count; i > 0; --i) {
      const double cap = std::sqrt(
        speed_[i] * speed_[i] + 2 * .7 * config_.limits.linear_acceleration * (s_[i] - s_[i - 1]));
      if (cap < speed_[i - 1]) {
        speed_[i - 1] = cap;
        reasons_[i - 1] = reasons_[i] | 64;
      }
    }
    for (std::size_t i = 1; i <= count; ++i) {
      const double cap = std::sqrt(
        speed_[i - 1] * speed_[i - 1] +
        2 * .7 * config_.limits.linear_acceleration * (s_[i] - s_[i - 1]));
      if (cap < speed_[i]) {
        speed_[i] = cap;
        reasons_[i] = reasons_[i - 1] | 64;
      }
    }
  }
  ReferenceProfile(const Input & input, const Config & config)
  : ReferenceProfile(
      input.path, config,
      free_heading(input) ? std::optional<double>(input.state.pose.yaw) : std::nullopt)
  {
    if (
      (free_heading(input) || input.reference_kind == ReferenceKind::EffectivePath) &&
      input.goal_active && distance(input.path.back(), input.goal) < .001 &&
      input.goal_heading_weight > 0) {
      const double remaining =
        std::max(ordered_remaining(input), distance(input.state.pose, input.goal));
      const double error = std::abs(wrap_angle(input.goal.yaw - input.state.pose.yaw));
      // Leave time to turn before arrival. The same cap governs nominal speed,
      // scoring, seed generation and retiming.
      const double cap = config.max_yaw_rate *
                         std::max(0., remaining - input.goal_tolerance.position) /
                         std::max(.01, error);
      for (std::size_t i = 0; i < speed_.size(); ++i)
        if (cap < speed_[i]) {
          speed_[i] = cap;
          reasons_[i] |= 128;
        }
    }
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
  double heading_preference(double arc) const
  {
    auto it = std::upper_bound(s_.begin(), s_.end(), arc);
    const auto i = static_cast<std::size_t>(it - s_.begin());
    if (!i) return preference_.front();
    if (i >= s_.size()) return preference_.back();
    const double t = (arc - s_[i - 1]) / (s_[i] - s_[i - 1]);
    return preference_[i - 1] + t * (preference_[i] - preference_[i - 1]);
  }
  // Bit mask: geometry/end=1, yaw=2, wheel speed=4, corner=8,
  // actuator rate=16, acceleration envelope=64, final heading approach=128. Interpolation combines neighboring active limits.
  std::uint32_t limiting_reason(double arc) const
  {
    auto it = std::upper_bound(s_.begin(), s_.end(), arc);
    const auto i = static_cast<std::size_t>(it - s_.begin());
    if (!i) return reasons_.front();
    if (i >= s_.size()) return reasons_.back();
    return reasons_[i - 1] | reasons_[i];
  }
  double next_branch(double arc) const
  {
    for (double branch : branches_)
      if (branch >= arc - .025) return std::max(0., branch - arc);
    return -1;
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
      -std::sin(body_yaw) * x + std::cos(body_yaw) * y, body_yaw_ ? 0 : path_.yaw_derivative(arc)};
  }

private:
  std::vector<Pose> points_;
  PathView path_;
  Config config_;
  std::optional<double> body_yaw_;
  std::vector<double> s_, speed_, corners_, branches_, preference_;
  std::vector<std::uint32_t> reasons_;
};
}  // namespace swerve_mppi
