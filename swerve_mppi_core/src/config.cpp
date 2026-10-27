#include "swerve_mppi/config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/types.hpp"

namespace swerve_mppi
{
double wrap_angle(double x) { return std::remainder(x, 2 * std::acos(-1.0)); }
double distance(const Pose & a, const Pose & b) { return std::hypot(a.x - b.x, a.y - b.y); }
bool GridView::valid() const
{
  return data && width && height && width <= std::numeric_limits<std::size_t>::max() / height &&
         std::isfinite(resolution) && resolution > 0 && std::isfinite(origin_x) &&
         std::isfinite(origin_y);
}
bool GridView::world_to_map(double x, double y, std::size_t & mx, std::size_t & my) const
{
  if (!valid() || !std::isfinite(x) || !std::isfinite(y)) return false;
  const double a = (x - origin_x) / resolution, b = (y - origin_y) / resolution;
  if (a < 0 || b < 0 || a >= static_cast<double>(width) || b >= static_cast<double>(height))
    return false;
  mx = static_cast<std::size_t>(a);
  my = static_cast<std::size_t>(b);
  return true;
}
void validate(const Config & c)
{
  swerve_chassis_core::validate(c.geometry, c.limits);
  for (double x : {c.max_yaw_rate, c.model_dt, c.integration_dt, c.temperature,
      c.lookahead, c.approach_distance, c.approach_gain, c.max_residual})
    if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("positive finite parameter required");
  for (double x : {c.max_speed, c.noise_linear, c.noise_angular})
    if (!std::isfinite(x) || x < 0) throw std::invalid_argument("nonnegative finite parameter required");
  if (!std::isfinite(c.noise_correlation) || c.noise_correlation < 0 || c.noise_correlation >= 1 ||
      c.horizon < 2 || c.horizon > 200 || c.samples < 4 || c.samples > 2048 ||
      c.iterations < 1 || c.iterations > 8 || c.integration_dt > c.model_dt ||
      c.integration_dt > 0.1 || c.model_dt / c.integration_dt > 100 ||
      c.max_speed > c.limits.linear_speed || c.max_yaw_rate > c.limits.angular_speed)
    throw std::invalid_argument("invalid optimizer dimensions, discretization or planner limits");
}
}  // namespace swerve_mppi
