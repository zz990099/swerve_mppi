#pragma once
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/critic.hpp"

namespace swerve_mppi_ros
{
// The ROS adapter supplies an immutable grid snapshot. Stage-one launch disables
// this critic; enable it explicitly when supplying a costmap.
class CostmapCritic final : public swerve_mppi::Critic
{
public:
  explicit CostmapCritic(double weight = 5.0, bool unknown_is_obstacle = true)
  : weight_(weight), unknown_(unknown_is_obstacle)
  {
    if (!std::isfinite(weight) || weight < 0) throw std::invalid_argument("invalid costmap weight");
  }
  void score(
    const swerve_mppi::CriticContext & c, const std::vector<swerve_mppi::Trajectory> & batch,
    std::vector<double> & costs) const override
  {
    if (!c.input.map.valid()) throw std::invalid_argument("costmap is missing");
    for (std::size_t k = 0; k < batch.size(); ++k)
      for (const auto & s : batch[k].states) {
        std::size_t x, y;
        if (!c.input.map.world_to_map(s.pose.x, s.pose.y, x, y)) {
          costs[k] = std::numeric_limits<double>::infinity();
          break;
        }
        auto cost = c.input.map.data[y * c.input.map.width + x];
        if (cost == 254 || (cost == 255 && unknown_)) {
          costs[k] = std::numeric_limits<double>::infinity();
          break;
        }
        if (cost != 255) costs[k] += weight_ * c.config.model_dt * cost / 254.0;
      }
  }

private:
  double weight_;
  bool unknown_;
};
}  // namespace swerve_mppi_ros
