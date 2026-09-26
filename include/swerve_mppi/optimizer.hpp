#pragma once

#include <limits>
#include <random>
#include <vector>

#include "swerve_mppi/model.hpp"

namespace swerve_mppi {

struct Branch {
  DriveMode mode = DriveMode::DualAckermann;
  std::size_t switch_step = 0;
  bool switches = false;
};

struct Solution {
  Branch branch;
  std::vector<Control> controls;
  double cost = std::numeric_limits<double>::infinity();
  std::size_t feasible_rollouts = 0;
};

class Optimizer {
 public:
  explicit Optimizer(const Config & config);

  std::vector<Branch> make_branches(const VehicleState & state) const;
  Solution optimize(const ControllerInput & input, const Branch & branch);
  void reset();

 private:
  double score(const ControllerInput & input, const Branch & branch,
               const std::vector<Control> & controls) const;
  std::vector<Control> seed(const ControllerInput & input,
                            const Branch & branch) const;
  Config config_;
  DriveModel model_;
  TransitionModel transition_;
  std::mt19937 rng_;
  std::vector<std::pair<Branch, std::vector<Control>>> warm_starts_;
};

}  // namespace swerve_mppi
