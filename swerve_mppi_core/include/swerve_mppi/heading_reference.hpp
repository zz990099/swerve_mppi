#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/path.hpp"

namespace swerve_mppi
{
// Prepare the complete external route before local cropping. Intermediate yaw
// and coincident-position yaw events have no meaning in an XY tracking task.
inline std::vector<Pose> geometric_path(const std::vector<Pose> & source, double lookahead)
{
  std::vector<Pose> points;
  for (auto p : source) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw))
      throw std::invalid_argument("invalid path pose");
    if (points.empty() || distance(points.back(), p) > 1e-8) points.push_back(p);
  }
  if (points.empty()) throw std::invalid_argument("empty path");
  if (points.size() == 1) return {source.back()};
  PathView view(points);
  for (std::size_t i = 0; i < points.size(); ++i) {
    double a = view.vertex_arc(i);
    double b = std::min(view.length(), a + lookahead);
    // Do not look through a mandatory geometric corner.
    b = std::min(b, view.next_corner(a));
    if (i + 1 == points.size()) a = std::max(view.vertex_arc(i - 1), a - lookahead);
    const auto p = view.at(a), q = view.at(b);
    points[i].yaw = std::atan2(q.y - p.y, q.x - p.x);
  }
  return points;
}
inline double ordered_remaining(const Input & input)
{
  if (input.remaining_arc >= 0) return input.remaining_arc;
  PathView path(input.path);
  return path.length() - path.start_match(input.state.pose).arc;
}
inline double terminal_heading_weight(const Input & input, const Config & config)
{
  if (input.reference_kind == ReferenceKind::FixedHeading) return 1;
  if (
    input.reference_kind == ReferenceKind::WheelAlignment ||
    input.reference_kind == ReferenceKind::Departure)
    return 0;
  if (!input.goal_active || distance(input.path.back(), input.goal) > .001) return 0;
  const double remaining = ordered_remaining(input);
  const double remaining_distance = std::max(remaining, distance(input.state.pose, input.goal));
  const double t = std::clamp(
    (config.goal_heading_start_distance - remaining_distance) /
      (config.goal_heading_start_distance - config.goal_heading_full_distance),
    0., 1.);
  return t * t * (3 - 2 * t);
}
// Ordinary XY tracking leaves body yaw free. Explicit internal references keep
// their heading semantics, independently of the external-path policy.
inline bool free_heading(const Input & input)
{
  return input.reference_kind == ReferenceKind::GeometricPath;
}
inline bool terminal_position_reached(const Input & input)
{
  return input.goal_active && distance(input.path.back(), input.goal) < .001 &&
         ordered_remaining(input) <= input.goal_tolerance.position &&
         distance(input.state.pose, input.goal) <= input.goal_tolerance.position;
}
inline bool inside_goal(const State & state, const Pose & goal, const GoalTolerance & t)
{
  return distance(state.pose, goal) <= t.position &&
         std::abs(wrap_angle(state.pose.yaw - goal.yaw)) <= t.yaw &&
         std::hypot(state.velocity.vx, state.velocity.vy) <= t.linear_speed &&
         std::abs(state.velocity.wz) <= t.angular_speed;
}
inline bool valid_goal_tolerance(const GoalTolerance & t)
{
  for (double x : {t.position, t.yaw, t.linear_speed, t.angular_speed, t.settle})
    if (!std::isfinite(x) || x <= 0) return false;
  return true;
}
inline double heading_feedback(double reference_yaw, double body_yaw)
{
  return wrap_angle(reference_yaw - body_yaw);
}
}  // namespace swerve_mppi
