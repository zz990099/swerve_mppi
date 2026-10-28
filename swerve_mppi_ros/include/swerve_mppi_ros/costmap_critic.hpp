#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/critic.hpp"
namespace swerve_mppi_ros
{
// Conservative circumscribed-footprint disk, swept along every predicted segment.
// This can reject narrow passages that a polygon could traverse; never shrinks footprint.
class CostmapCritic final : public swerve_mppi::Critic
{
public:
  explicit CostmapCritic(double weight = 5, bool unknown_is_obstacle = true, double radius = 0)
  : weight_(weight), radius_(radius), unknown_(unknown_is_obstacle)
  {
    if (!std::isfinite(weight) || weight < 0 || !std::isfinite(radius) || radius < 0)
      throw std::invalid_argument("invalid costmap critic configuration");
  }
  void score(
    const swerve_mppi::CriticContext & c, const std::vector<swerve_mppi::Trajectory> & batch,
    std::vector<double> & costs) const override
  {
    const auto & map = c.input.map;
    if (!map.valid()) throw std::invalid_argument("costmap missing");
    // Empty maps need only the swept footprint bounds. A rectangle is convex,
    // so segment endpoints suffice; occupied/unknown maps retain full sweep tests.
    const bool all_free = std::all_of(
      map.data, map.data + map.width * map.height, [](std::uint8_t v) { return v == 0; });
    if (all_free) {
      for (std::size_t k = 0; k < batch.size(); ++k) {
        if (!batch[k].valid) continue;
        for (const auto & state : batch[k].states) {
          std::size_t x, y;
          if (
            !map.world_to_map(state.pose.x - radius_, state.pose.y - radius_, x, y) ||
            !map.world_to_map(state.pose.x + radius_, state.pose.y + radius_, x, y)) {
            costs[k] = std::numeric_limits<double>::infinity();
            break;
          }
        }
      }
      return;
    }
    auto evaluate = [&](double x, double y) {
      std::size_t x0, y0, x1, y1;
      if (
        !map.world_to_map(x - radius_, y - radius_, x0, y0) ||
        !map.world_to_map(x + radius_, y + radius_, x1, y1))
        return std::numeric_limits<double>::infinity();
      double maximum = 0;
      for (std::size_t iy = y0; iy <= y1; ++iy)
        for (std::size_t ix = x0; ix <= x1; ++ix) {
          const double cx = map.origin_x + (ix + 0.5) * map.resolution,
                       cy = map.origin_y + (iy + 0.5) * map.resolution;
          if (std::hypot(cx - x, cy - y) > radius_ + map.resolution * 0.707107) continue;
          const auto cost = map.data[iy * map.width + ix];
          if (cost == 253 || cost == 254 || (cost == 255 && unknown_))
            return std::numeric_limits<double>::infinity();
          if (cost != 255) maximum = std::max(maximum, cost / 254.0);
        }
      return maximum;
    };
    for (std::size_t k = 0; k < batch.size(); ++k) {
      const auto & states = batch[k].states;
      if (!batch[k].valid) continue;
      for (std::size_t i = 0; i < states.size(); ++i) {
        const auto & a = states[i ? i - 1 : i].pose;
        const auto & b = states[i].pose;
        const double length = swerve_mppi::distance(a, b);
        if (!std::isfinite(length) || length / map.resolution > 10000) {
          costs[k] = std::numeric_limits<double>::infinity();
          break;
        }
        const int steps = std::max(1, static_cast<int>(std::ceil(length / (map.resolution * 0.5))));
        double worst = 0;
        for (int s = 0; s <= steps; ++s) {
          const double f = static_cast<double>(s) / steps;
          worst = std::max(worst, evaluate(a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f));
        }
        if (!std::isfinite(worst))
          costs[k] = std::numeric_limits<double>::infinity();
        else
          costs[k] += weight_ * c.config.model_dt * worst;
        if (!std::isfinite(costs[k])) break;
      }
    }
  }

private:
  double weight_, radius_;
  bool unknown_;
};
}  // namespace swerve_mppi_ros
