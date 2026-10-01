#pragma once

#include "swerve_mppi/rollout.hpp"
#include <limits>

namespace swerve_mppi {
struct Solution {
  Branch branch;
  std::vector<Control> controls;
  Trajectory trajectory;
  double cost = std::numeric_limits<double>::infinity();
  std::size_t feasible_rollouts = 0;
  PlanningStats planning_stats;
};
class ModeScheduler {
public:
  explicit ModeScheduler(const Config &config);
  std::vector<Branch> make_branches(const VehicleState &state) const;
  // The first solution must be the keep-mode solution. Rejected switches never win.
  std::size_t select(const std::vector<Solution> &solutions) const;

private:
  Config config_;
};
class ModeManager {
public:
  explicit ModeManager(const Config &config);
  void begin(DriveMode target_mode, const Control &entry_intent, const VehicleState &observed);
  bool active() const { return phase_ != TransitionPhase::Stable; }
  TransitionPhase phase() const { return phase_; }
  Output update(const VehicleState &observed);
  void reset();

private:
  Config config_;
  TransitionPhase phase_ = TransitionPhase::Stable;
  ModeRequest request_;
  std::uint64_t last_request_id_ = 0;
  double start_s_ = 0.0;
  double last_stamp_s_ = -1.0;
};
} // namespace swerve_mppi
