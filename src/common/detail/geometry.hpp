#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/common/types.hpp"

namespace swerve_mppi::detail
{
inline double finite_geometry(double value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument("nonfinite derived geometry");
  }
  return value;
}
inline double segment_projection(
  const Pose2d & point, const Pose2d & from, double dx, double dy, double length)
{
  const double x = finite_geometry(point.x - from.x);
  const double y = finite_geometry(point.y - from.y);
  const double scale = std::max(std::abs(x), std::abs(y));
  if (scale == 0) {
    return 0;
  }
  const double dot = (x / scale) * (dx / length) + (y / scale) * (dy / length);
  const double ratio = scale / length;
  return finite_geometry(std::isfinite(ratio) ? dot * ratio : (dot * scale) / length);
}
struct SegmentDistance
{
  double distance;
  double fraction;
};
inline SegmentDistance segment_distance(
  const Pose2d & point, const Pose2d & from, const Pose2d & to)
{
  const double dx = finite_geometry(to.x - from.x), dy = finite_geometry(to.y - from.y);
  const double length2 = finite_geometry(dx * dx + dy * dy);
  const double x = finite_geometry(point.x - from.x), y = finite_geometry(point.y - from.y);
  const double dot = x * dx + y * dy;
  double t = 0;
  if (length2 > 0 && std::isfinite(length2) && std::isfinite(dot)) {
    t = std::clamp(dot / length2, 0.0, 1.0);
  } else {
    const double length = finite_geometry(std::hypot(dx, dy));
    if (length > 0) {
      t = std::clamp(segment_projection(point, from, dx, dy, length), 0.0, 1.0);
    }
  }
  // Subtract before multiplying so extreme representable projections do not
  // lose the small displacement to a large world origin.
  return {finite_geometry(std::hypot(x - t * dx, y - t * dy)), t};
}
inline double geometry_turn(double ax, double ay, double bx, double by)
{
  const double a = finite_geometry(std::hypot(ax, ay));
  const double b = finite_geometry(std::hypot(bx, by));
  if (a == 0 || b == 0) {
    return 0;
  }
  return finite_geometry(
    std::atan2(
      (ax / a) * (by / b) - (ay / a) * (bx / b), (ax / a) * (bx / b) + (ay / a) * (by / b)));
}
}  // namespace swerve_mppi::detail
