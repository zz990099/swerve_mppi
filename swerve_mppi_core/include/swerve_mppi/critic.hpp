#pragma once
#include <memory>
#include <vector>

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/types.hpp"

namespace swerve_mppi
{
class ReferenceProfile;
struct CriticContext
{
  const Input & input;
  const Config & config;
  const ReferenceProfile * reference = nullptr;
  std::vector<TrackingCost> * breakdown = nullptr;
  const Velocity * initial_intent = nullptr;
  bool alignment_only = false;
};
class Critic
{
public:
  virtual ~Critic() = default;
  virtual void prepare(const CriticContext &) {}
  virtual Rejection rejection_reason() const { return Rejection::Critic; }
  // Add costs for the whole batch. Infinity rejects a candidate.
  virtual void score(
    const CriticContext &, const std::vector<Trajectory> &, std::vector<double> & costs) const = 0;
};
using Critics = std::vector<std::shared_ptr<Critic>>;
struct TrackingWeights
{
  double path_distance = 1.0, path_heading = 0.5, progress = 1.0;
  double goal_position = 2.0, goal_heading = 1.0;
  double effort = 0.005, smoothness = 0.1, steering_change = 0.05, consistency = 1.0;
  double reference_velocity = 1.0;
  double presteering_alignment = 1.0;
};
// Pure numerical defaults; ROS costmap critics implement the same interface.
Critics tracking_critics(TrackingWeights weights = {});
}  // namespace swerve_mppi
