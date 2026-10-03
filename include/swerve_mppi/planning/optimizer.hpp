#pragma once

#include "swerve_mppi/common/planning_budget.hpp"
#include "swerve_mppi/planning/critics.hpp"
#include "swerve_mppi/planning/mode.hpp"
#include "swerve_mppi/planning/noise.hpp"

namespace swerve_mppi
{
class Optimizer
{
public:
  explicit Optimizer(
    const Config & config, std::shared_ptr<const TrajectoryValidator> validator = nullptr);
  Solution optimize(
    const ControllerInput & input, const Branch & branch, const PlanningBudget * budget = nullptr);
  // Advance a warm start only after the controller actually issues its drive
  // action.
  void accept(const Solution & solution, DriveMode mode);
  // Discard a stale proposal without replaying previously sampled noise.
  void clear_warm_start();
  // Explicit deterministic restart, including the configured random seed.
  void reset();
  CriticManager & critics() { return critics_; }

private:
  std::vector<Control> seed(
    const ControllerInput & input, const Branch & branch, bool use_warm = true) const;
  struct Sample
  {
    std::vector<Control> noise;
    std::vector<bool> active;
    double cost = std::numeric_limits<double>::infinity();
  };
  std::vector<Sample> samples_;
  std::vector<Control> candidate_;
  std::vector<Control> weighted_;
  Trajectory proposal_;
  Config config_;
  DriveModel model_;
  RolloutEngine rollout_;
  CriticManager critics_;
  NoiseGenerator noise_;
  DriveMode warm_mode_ = DriveMode::DualAckermann;
  std::vector<Control> warm_start_;
};
}  // namespace swerve_mppi
