#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "swerve_mppi/types.hpp"
namespace swerve_mppi
{
struct PathMatch
{
  Pose pose;
  double arc = 0, error = 0, tx = 0, ty = 0;
  std::size_t segment = 0;
};
// Arc length, not waypoint count, defines progress and interpolation.
class PathView
{
public:
  explicit PathView(const std::vector<Pose> & points) : p_(points), arc_(points.size(), 0)
  {
    for (std::size_t i = 1; i < p_.size(); ++i) arc_[i] = arc_[i - 1] + distance(p_[i - 1], p_[i]);
  }
  double length() const { return arc_.back(); }
  double vertex_arc(std::size_t i) const { return arc_[i]; }
  Pose at(double s) const
  {
    if (s >= length() - 1e-10) return p_.back();
    auto it = std::upper_bound(arc_.begin(), arc_.end(), std::max(0., s));
    std::size_t i = std::min(p_.size() - 1, static_cast<std::size_t>(it - arc_.begin()));
    if (!i) return p_.front();
    double t = (s - arc_[i - 1]) / std::max(1e-12, arc_[i] - arc_[i - 1]);
    return {
      p_[i - 1].x + t * (p_[i].x - p_[i - 1].x), p_[i - 1].y + t * (p_[i].y - p_[i - 1].y),
      wrap_angle(p_[i - 1].yaw + t * wrap_angle(p_[i].yaw - p_[i - 1].yaw))};
  }
  PathMatch match(
    Pose pose, double lo = 0, double hi = std::numeric_limits<double>::infinity()) const
  {
    PathMatch best{p_.back(), length(), distance(pose, p_.back()), 0, 0, p_.size() - 1};
    double error = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < p_.size(); ++i) {
      double len = arc_[i] - arc_[i - 1];
      if (len < 1e-10) continue;
      double a = std::max(lo, arc_[i - 1]), b = std::min(hi, arc_[i]);
      if (b < a - 1e-10) continue;
      double tx = (p_[i].x - p_[i - 1].x) / len, ty = (p_[i].y - p_[i - 1].y) / len;
      double s = std::clamp(
        arc_[i - 1] + (pose.x - p_[i - 1].x) * tx + (pose.y - p_[i - 1].y) * ty, a, std::max(a, b));
      auto q = at(s);
      double e = distance(pose, q);
      if (
        e < error - 1e-10 ||
        (std::abs(e - error) < 1e-10 && std::abs(s - lo) < 1e-8 && arc_[i - 1] >= lo - 1e-8)) {
        error = e;
        best = {q, s, e, tx, ty, i};
      }
    }
    return best;
  }
  double turn(std::size_t i) const
  {
    if (!i || i + 1 >= p_.size()) return 0;
    double ax = p_[i].x - p_[i - 1].x, ay = p_[i].y - p_[i - 1].y, bx = p_[i + 1].x - p_[i].x,
           by = p_[i + 1].y - p_[i].y;
    if (std::hypot(ax, ay) < 1e-8 || std::hypot(bx, by) < 1e-8) return 0;
    return std::abs(std::atan2(ax * by - ay * bx, ax * bx + ay * by));
  }
  double next_corner(double s) const
  {
    for (std::size_t i = 1; i + 1 < p_.size(); ++i)
      if (arc_[i] > s + 1e-5 && turn(i) >= 1.0471975512) return arc_[i];
    return length();
  }
  double speed_limit(
    double s, double maximum, double acceleration, double gain, double steer_rate) const
  {
    double speed = std::min(maximum, gain * std::max(0., length() - s));
    for (std::size_t i = 1; i + 1 < p_.size(); ++i) {
      double d = arc_[i] - s;
      if (d < 1e-5) continue;
      double angle = turn(i), vertex_speed = maximum;
      if (angle >= 1.0471975512)
        vertex_speed = 0;
      else {
        double curvature = angle / std::max(.01, .5 * (arc_[i + 1] - arc_[i - 1]));
        if (curvature > 1e-6)
          vertex_speed =
            std::min({maximum, std::sqrt(.6 * acceleration / curvature), steer_rate / curvature});
      }
      speed = std::min(speed, std::sqrt(vertex_speed * vertex_speed + 2 * .7 * acceleration * d));
      if (vertex_speed == 0) speed = std::min(speed, gain * d);
    }
    return speed;
  }
  double yaw_derivative(double s) const
  {
    double a = std::max(0., s - .02), b = std::min(length(), s + .02);
    return b > a + 1e-6 ? wrap_angle(at(b).yaw - at(a).yaw) / (b - a) : 0;
  }

private:
  const std::vector<Pose> & p_;
  std::vector<double> arc_;
};
}  // namespace swerve_mppi
