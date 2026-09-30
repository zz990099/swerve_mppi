#include "swerve_mppi/noise.hpp"
#include <stdexcept>

namespace swerve_mppi {
NoiseGenerator::NoiseGenerator(const Config &config)
    : config_(config), model_(config), rng_(config.random_seed) {}
void NoiseGenerator::sample(const std::vector<Control> &mean, const Branch &branch,
                            DriveMode current, std::vector<Control> &candidate,
                            std::vector<Control> &noise) {
  candidate.resize(mean.size());
  noise.resize(mean.size());
  for (std::size_t t = 0; t < mean.size(); ++t) {
    const DriveMode mode = branch.switches && t >= branch.switch_step ? branch.mode : current;
    const Control sampled{mean[t].vx + config_.noise_v_mps * normal_(rng_),
                          mean[t].vy + config_.noise_v_mps * normal_(rng_),
                          mean[t].wz + config_.noise_w_radps * normal_(rng_)};
    candidate[t] = model_.project(sampled, mode);
    noise[t] = {candidate[t].vx - mean[t].vx, candidate[t].vy - mean[t].vy,
                candidate[t].wz - mean[t].wz};
  }
}
double NoiseGenerator::correction(const std::vector<Control> &mean,
                                  const std::vector<Control> &noise,
                                  const std::vector<bool> &active) const {
  if (noise.size() != mean.size() || active.size() != mean.size())
    throw std::invalid_argument("noise correction dimensions must match the nominal sequence");
  double sum = 0.0;
  for (std::size_t t = 0; t < mean.size(); ++t) {
    if (!active[t])
      continue;
    if (config_.noise_v_mps > 0.0)
      sum += (mean[t].vx * noise[t].vx + mean[t].vy * noise[t].vy) /
             (config_.noise_v_mps * config_.noise_v_mps);
    if (config_.noise_w_radps > 0.0)
      sum += mean[t].wz * noise[t].wz / (config_.noise_w_radps * config_.noise_w_radps);
  }
  return config_.control_correction_weight * sum;
}
void NoiseGenerator::reset() {
  rng_.seed(config_.random_seed);
  normal_.reset();
}
} // namespace swerve_mppi
