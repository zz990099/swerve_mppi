#pragma once

#include <limits>

#include "swerve_mppi/model/rollout.hpp"

namespace swerve_mppi
{
struct Solution
{
  Branch branch;
  std::vector<Control> controls;
  Trajectory trajectory;
  double cost = std::numeric_limits<double>::infinity();
  std::size_t feasible_rollouts = 0;
  PlanningStats planning_stats;
};
class ModeScheduler
{
public:
  explicit ModeScheduler(const Config & config);
  std::vector<Branch> make_branches(const VehicleState & state) const;
  // The first solution must be the keep-mode solution. Rejected switches never
  // win.
  std::size_t select(const std::vector<Solution> & solutions) const;

private:
  Config config_;
};
}  // namespace swerve_mppi
