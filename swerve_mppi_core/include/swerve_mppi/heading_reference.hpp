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
inline double terminal_heading_weight(const Input & input, const Config & config)
{
  if (input.reference_kind == ReferenceKind::FixedHeading) return 1;
  if (
    input.reference_kind == ReferenceKind::WheelAlignment ||
    input.reference_kind == ReferenceKind::Departure)
    return 0;
  if (!input.goal_active || distance(input.path.back(), input.goal) > .001) return 0;
  PathView path(input.path);
  const double remaining = input.remaining_arc >= 0
                             ? input.remaining_arc
                             : path.length() - path.start_match(input.state.pose).arc;
  const double remaining_distance = std::max(remaining, distance(input.state.pose, input.goal));
  const double t = std::clamp(
    (config.goal_heading_start_distance - remaining_distance) /
      (config.goal_heading_start_distance - config.goal_heading_full_distance),
    0., 1.);
  return t * t * (3 - 2 * t);
}
inline double heading_feedback(const Input & input, double travel_yaw, double body_yaw)
{
  const double a = input.goal_heading_weight;
  return (1 - a) * wrap_angle(travel_yaw - body_yaw) + a * wrap_angle(input.goal.yaw - body_yaw);
}
}  // namespace swerve_mppi
