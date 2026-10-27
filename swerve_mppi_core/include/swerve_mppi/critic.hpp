#pragma once
#include <memory>
#include <vector>

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/types.hpp"

namespace swerve_mppi
{
struct CriticContext
{
  const Input & input;
  const Config & config;
};
class Critic
{
public:
  virtual ~Critic() = default;
  virtual void prepare(const CriticContext &) {}
  // Add costs for the whole batch. Infinity rejects a candidate.
  virtual void score(
    const CriticContext &, const std::vector<Trajectory> &, std::vector<double> & costs) const = 0;
};
using Critics = std::vector<std::shared_ptr<Critic>>;
struct TrackingWeights
{
  double path_distance = 4.0, path_heading = 0.3, progress = 8.0;
  double goal_position = 30.0, goal_heading = 12.0;
  double effort = 0.01, smoothness = 0.08, steering_change = 0.02, consistency = 1.0;
};
// Pure numerical defaults; ROS costmap critics implement the same interface.
Critics tracking_critics(TrackingWeights weights = {});
}  // namespace swerve_mppi
