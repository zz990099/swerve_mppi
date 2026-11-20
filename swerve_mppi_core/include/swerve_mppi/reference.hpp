#pragma once
#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <swerve_chassis_core/kinematics.hpp>
#include <vector>

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/path.hpp"

namespace swerve_mppi
{
struct HeadingMatch
{
  double error;
  double loss;
};

inline HeadingMatch match_geometric_heading(double yaw, double tangent, const Config & config)
{
  auto loss = [&](double error) {
    const double x = std::abs(error) / config.geometric_heading_scale;
    return x <= 1 ? x * x : 2 * x - 1;
  };
  const double forward = wrap_angle(tangent - yaw);
  const double forward_loss = loss(forward);
  if (config.geometric_heading_allow_reverse) {
    const double reverse = wrap_angle(tangent + swerve_chassis_core::pi - yaw);
    // Prefer forward within a small band around equal turning effort.
    const double reverse_loss = loss(reverse) + 0.35;
    if (reverse_loss < forward_loss) return {reverse, reverse_loss};
  }
  return {forward, forward_loss};
}

inline bool prefers_heading_alignment(
  double strength, HeadingMatch match, double clear_arc, const Config & config)
{
  return config.samples >= 6 && clear_arc >= 1.5 * config.geometric_heading_window &&
         strength > .8 && std::abs(match.error) > .45;
}

// Adaptive time parameterization of an ordered geometric path. Intermediate
// waypoint yaw is intentionally ignored: body heading is free during travel and
// the terminal pose yaw is handled by the goal critic. The shared envelope is
// geometric; only local previews have a measured body frame for wheel branches.
class ReferenceProfile
{
public:
  struct HeadingPreference
  {
    double yaw = 0, strength = 0, curvature = 0;
  };
  ReferenceProfile(
    const std::vector<Pose> & points, const Config & config, SpeedPreview preview = {},
    double entry_arc = 0, double dt = .05, const State * chassis_state = nullptr)
  : points_(points),
    path_(points_),
    config_(config),
    preview_(preview),
    entry_arc_(entry_arc),
    acceleration_(.7 * config.limits.linear_acceleration),
    entry_speed_(preview.previous_speed.value_or(0.) + acceleration_ * dt)
  {
    validate(config_);
    if (points.empty() || points.size() > 4096 || !std::isfinite(path_.length()))
      throw std::invalid_argument("invalid reference path");
    if (
      !std::isfinite(preview_.start_arc) || preview_.start_arc < 0 ||
      (preview_.previous_speed &&
       (!std::isfinite(*preview_.previous_speed) || *preview_.previous_speed < 0)))
      throw std::invalid_argument("invalid speed preview");
    for (std::size_t i = 1; i + 1 < points.size(); ++i)
      if (path_.turn(i) >= PathView::stop_angle) corners_.push_back(path_.vertex_arc(i));
    // The stop/reorientation controller restarts from a cropped outgoing leg.
    // A shared stationary knot must not pin its target speed after the stop.
    if (preview_.plan && preview_.plan->near_corner(preview_.start_arc + entry_arc_)) preview_ = {};
    // Keep the same knot grid across roundoff in rigidly transformed paths.
    auto count =
      static_cast<std::size_t>(std::clamp(std::ceil(path_.length() / .025 - 1e-9), 1., 4096.));
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
    std::vector<Velocity> tangents(count + 1);
    std::vector<swerve_chassis_core::Joints> wheels(chassis_state ? count + 1 : 0);
    auto previous = chassis_state ? chassis_state->steering : std::array<double, 4>{};
    const double wheel_cap = config_.limits.wheel_speed * config_.geometry.radius;
    for (std::size_t i = 0; i <= count; ++i) {
      const double s = s_[i];
      tangents[i] = unit(s, 0);
      if (chassis_state) {
        const auto u = rotate_velocity(tangents[i], -chassis_state->pose.yaw);
        wheels[i] = swerve_chassis_core::inverse(config_.geometry, {u.vx, u.vy, u.wz}, previous);
        previous = wheels[i].steering;
      }
      double cap = preview_.plan ? config_.max_speed
                                 : path_.speed_limit(
                                     s, config_.max_speed, config_.limits.linear_acceleration,
                                     config_.approach_gain, config_.limits.steering_rate);
      if (wheel_cap < cap) {
        cap = wheel_cap;
        reasons_[i] = 4;
      }
      for (double corner : corners_)
        if (std::abs(s - corner) < 1e-8) {
          cap = 0;
          reasons_[i] = 8;
        }
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
      if (!preview_.plan) {
        const auto & a = tangents[i - 1];
        const auto & b = tangents[i];
        const double turn =
          std::abs(std::atan2(a.vx * b.vy - a.vy * b.vx, a.vx * b.vx + a.vy * b.vy));
        cap = std::min(cap, steering_rate * ds / std::max(1e-9, turn));
      }
      if (chassis_state) {
        for (std::size_t w = 0; w < 4; ++w) {
          const double delta = std::abs(wheels[i].steering[w] - wheels[i - 1].steering[w]);
          if (delta <= swerve_chassis_core::pi / 2) continue;
          if (branches_.empty() || branches_.back() != s_[i - 1]) branches_.push_back(s_[i - 1]);
          // A true branch change needs braking before reorientation. Bound
          // the signed wheel-speed change only in this measured local frame.
          const double dq = std::abs(wheels[i].speed[w] - wheels[i - 1].speed[w]) / ds;
          cap =
            std::min(cap, std::sqrt(.5 * config_.limits.wheel_acceleration / std::max(1e-9, dq)));
        }
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
      const double cap = std::sqrt(speed_[i] * speed_[i] + 2 * acceleration_ * (s_[i] - s_[i - 1]));
      if (cap < speed_[i - 1]) {
        speed_[i - 1] = cap;
        reasons_[i - 1] = reasons_[i] | 64;
      }
    }
    for (std::size_t i = 1; i <= count; ++i) {
      const double cap =
        std::sqrt(speed_[i - 1] * speed_[i - 1] + 2 * acceleration_ * (s_[i] - s_[i - 1]));
      if (cap < speed_[i]) {
        speed_[i] = cap;
        reasons_[i] = reasons_[i - 1] | 64;
      }
    }
    // Shared immutable geometry is prepared once per compute, not per sample.
    // Uniform arc samples make the preference insensitive to waypoint density.
    headings_.reserve(s_.size());
    for (double arc : s_) headings_.push_back(make_heading(arc));
  }
  ReferenceProfile(const ReferenceProfile &) = delete;
  ReferenceProfile & operator=(const ReferenceProfile &) = delete;
  double speed(double arc) const
  {
    if (path_.length() < 1e-8) return 0;
    return std::min(unreleased_speed(arc), release_limit(arc));
  }
  bool near_corner(double arc) const
  {
    const auto it =
      std::lower_bound(corners_.begin(), corners_.end(), arc - config_.approach_distance);
    return it != corners_.end() && *it <= arc + config_.approach_distance;
  }
  bool just_passed_corner(double arc) const
  {
    const auto next = std::upper_bound(corners_.begin(), corners_.end(), arc + 1e-8);
    return next != corners_.begin() && arc - *std::prev(next) <= .25;
  }
  // Bit mask: geometry/end=1, wheel speed=4, corner=8, actuator rate=16,
  // acceleration envelope=64, rate of release across control cycles=128.
  std::uint32_t limiting_reason(double arc) const
  {
    if (release_limit(arc) < unreleased_speed(arc)) return 128;
    if (preview_.plan) {
      const double plan_speed = preview_.plan->speed(preview_.start_arc + arc);
      if (preview_end_limit(arc) < std::min(plan_speed, knot_speed(arc))) return 1;
      if (plan_speed <= knot_speed(arc))
        return preview_.plan->limiting_reason(preview_.start_arc + arc);
    }
    return knot_reason(arc);
  }

private:
  double preview_end_limit(double arc) const
  {
    return config_.approach_gain * std::max(0., path_.length() - arc);
  }
  double release_limit(double arc) const
  {
    if (!preview_.previous_speed) return config_.max_speed;
    return std::sqrt(
      entry_speed_ * entry_speed_ + 2 * acceleration_ * std::max(0., arc - entry_arc_));
  }
  double unreleased_speed(double arc) const
  {
    if (preview_.plan) {
      // A measured local branch can tighten the reusable geometric envelope.
      return std::min(
        {preview_.plan->speed(preview_.start_arc + arc), preview_end_limit(arc), knot_speed(arc)});
    }
    return knot_speed(arc);
  }
  double knot_speed(double arc) const
  {
    auto it = std::upper_bound(s_.begin(), s_.end(), arc);
    const auto i = static_cast<std::size_t>(it - s_.begin());
    if (!i) return speed_.front();
    if (i >= s_.size()) return speed_.back();
    const double t = (arc - s_[i - 1]) / (s_[i] - s_[i - 1]);
    return speed_[i - 1] + t * (speed_[i] - speed_[i - 1]);
  }
  std::uint32_t knot_reason(double arc) const
  {
    auto it = std::upper_bound(s_.begin(), s_.end(), arc);
    const auto i = static_cast<std::size_t>(it - s_.begin());
    if (!i) return reasons_.front();
    if (i >= s_.size()) return reasons_.back();
    return reasons_[i - 1] | reasons_[i];
  }

public:
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
      -std::sin(body_yaw) * x + std::cos(body_yaw) * y, 0};
  }
  HeadingPreference heading(double arc, bool preview_reaches_goal = false) const
  {
    if (path_.length() < 1e-8 || config_.geometric_heading_weight == 0) return {};
    const auto it = std::upper_bound(s_.begin(), s_.end(), arc);
    const auto i = static_cast<std::size_t>(it - s_.begin());
    auto out = i == 0 ? headings_.front() : headings_.back();
    if (i > 0 && i < s_.size()) {
      const double u = std::clamp((arc - s_[i - 1]) / (s_[i] - s_[i - 1]), 0., 1.);
      const auto a = headings_[i - 1], b = headings_[i];
      out = {
        wrap_angle(a.yaw + u * wrap_angle(b.yaw - a.yaw)),
        a.strength + u * (b.strength - a.strength), a.curvature + u * (b.curvature - a.curvature)};
    }
    if (preview_reaches_goal) {
      const double u = std::clamp(
        (path_.length() - arc - config_.goal_heading_activation_distance) /
          config_.geometric_heading_window,
        0., 1.);
      out.strength *= u * u * (3 - 2 * u);
    }
    return out;
  }

private:
  HeadingPreference make_heading(double arc) const
  {
    if (path_.length() < 1e-8 || config_.geometric_heading_weight == 0) return {};
    const double half = .5 * config_.geometric_heading_window;
    const double lo = std::max(0., arc - half), hi = std::min(path_.length(), arc + half);
    Pose previous = path_.at(lo);
    double x = 0, y = 0, length = 0, turn = 0, signed_turn = 0, previous_angle = 0;
    bool have_angle = false;
    for (int j = 1; j <= 16; ++j) {
      const auto point = path_.at(lo + (hi - lo) * j / 16);
      const double dx = point.x - previous.x, dy = point.y - previous.y;
      const double ds = std::hypot(dx, dy);
      if (ds > 1e-9) {
        const double angle = std::atan2(dy, dx);
        if (have_angle) {
          const double delta = wrap_angle(angle - previous_angle);
          turn += std::abs(delta);  // Opposite turns in an S must not cancel.
          signed_turn += delta;
        }
        previous_angle = angle;
        have_angle = true;
        x += dx;
        y += dy;
        length += ds;
      }
      previous = point;
    }
    const double chord = std::hypot(x, y);
    if (length < 1e-8 || chord < 1e-8) return {};
    const double coherence = std::clamp(chord / length, 0., 1.);
    const double complexity = turn / config_.geometric_heading_turn_scale;
    const double curvature = signed_turn / std::max(1e-8, hi - lo);
    const double demand = speed(arc) * std::abs(curvature) / config_.max_yaw_rate;
    return {
      std::atan2(y, x),
      coherence * coherence / ((1 + std::pow(complexity, 4)) * (1 + std::pow(demand, 4))),
      curvature};
  }
  std::vector<Pose> points_;
  PathView path_;
  Config config_;
  SpeedPreview preview_;
  double entry_arc_, acceleration_, entry_speed_;
  std::vector<double> s_, speed_, corners_, branches_;
  std::vector<std::uint32_t> reasons_;
  std::vector<HeadingPreference> headings_;
};
}  // namespace swerve_mppi
