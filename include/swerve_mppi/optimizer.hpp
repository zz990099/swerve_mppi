#pragma once

#include "swerve_mppi/critics.hpp"
#include "swerve_mppi/mode.hpp"
#include "swerve_mppi/noise.hpp"

namespace swerve_mppi {
class Optimizer {
public:
  explicit Optimizer(const Config &config);
  Solution optimize(const ControllerInput &input, const Branch &branch);
  // Advance a warm start only after the controller actually issues its drive action.
  void accept(const Solution &solution, DriveMode mode);
  void reset();
  CriticManager &critics() { return critics_; }

private:
  std::vector<Control> seed(const ControllerInput &input, const Branch &branch) const;
  Config config_;
  DriveModel model_;
  RolloutEngine rollout_;
  CriticManager critics_;
  NoiseGenerator noise_;
  DriveMode warm_mode_ = DriveMode::DualAckermann;
  std::vector<Control> warm_start_;
};
} // namespace swerve_mppi
