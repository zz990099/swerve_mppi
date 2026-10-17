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
bool valid_mode(Mode mode) { return static_cast<unsigned>(mode) <= 2; }
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
  for (double x :
       {c.wheelbase,       c.track,        c.wheel_radius,  c.max_speed,           c.max_yaw_rate,
        c.min_turn_radius, c.linear_accel, c.angular_accel, c.max_wheel_speed,     c.wheel_accel,
        c.steer_rate,      c.steer_limit,  c.realign_angle, c.alignment_tolerance, c.model_dt,
        c.integration_dt,  c.temperature,  c.lookahead,     c.approach_distance,   c.approach_gain})
    if (!std::isfinite(x) || x <= 0)
      throw std::invalid_argument("positive finite model/optimizer parameter required");
  for (double x :
       {c.alignment_dwell, c.stopped_wheel_speed, c.minimum_mode_dwell, c.switch_penalty,
        c.switch_hysteresis, c.noise_linear, c.noise_angular})
    if (!std::isfinite(x) || x < 0)
      throw std::invalid_argument("nonnegative finite parameter required");
  if (
    !std::isfinite(c.noise_correlation) || c.noise_correlation < 0 || c.noise_correlation >= 1 ||
    c.horizon < 2 || c.horizon > 200 || c.samples < 2 || c.samples > 2048 || c.iterations < 1 ||
    c.iterations > 8 || c.integration_dt > c.model_dt || c.model_dt / c.integration_dt > 100 ||
    c.steer_limit > std::acos(-1.0) / 2 + 1e-9)
    throw std::invalid_argument("invalid optimizer size or model discretization");
}
}  // namespace swerve_mppi
