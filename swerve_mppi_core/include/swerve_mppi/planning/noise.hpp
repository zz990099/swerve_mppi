#pragma once

#include <random>

#include "swerve_mppi/model/rollout.hpp"

namespace swerve_mppi
{
class NoiseGenerator
{
public:
  explicit NoiseGenerator(const Config & config);
  void sample(
    const std::vector<Control> & mean, const Branch & branch, DriveMode current_mode,
    std::vector<Control> & candidate, std::vector<Control> & effective_noise);
  // Marginal AR(1) precision over active ticks; only explicit entry resets it.
  // Pass the same branch used by sample() when a mode switch is present.
  double correction(
    const std::vector<Control> & mean, const std::vector<Control> & effective_noise,
    const std::vector<bool> & active, const Branch & branch = {}) const;
  void reset();

private:
  Config config_;
  DriveModel model_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
};
}  // namespace swerve_mppi
