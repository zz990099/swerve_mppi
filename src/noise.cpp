#include "swerve_mppi/noise.hpp"
#include <cmath>
#include <stdexcept>

namespace swerve_mppi {
NoiseGenerator::NoiseGenerator(const Config &config)
    : config_(config), model_(config), rng_(config.random_seed) {}
void NoiseGenerator::sample(const std::vector<Control> &mean, const Branch &branch,
                            DriveMode current, std::vector<Control> &candidate,
                            std::vector<Control> &noise) {
  candidate.resize(mean.size());
  noise.resize(mean.size());
  Control previous;
  bool restart = true;
  const double rho = config_.noise_correlation;
  for (std::size_t t = 0; t < mean.size(); ++t) {
    // Entry intent defines the frozen discrete request, not a drive tick.
    // Keep it identical across proposals so masked noise cannot change alignment.
    if (branch.switches && t == branch.switch_step) {
      candidate[t] = mean[t];
      noise[t] = {};
      restart = true;
      continue;
    }
    const DriveMode mode = branch.switches && t >= branch.switch_step ? branch.mode : current;
    const double a = restart ? 0.0 : rho;
    const double b = restart ? 1.0 : std::sqrt(1.0 - rho * rho);
    previous = {a * previous.vx + b * normal_(rng_), a * previous.vy + b * normal_(rng_),
                a * previous.wz + b * normal_(rng_)};
    restart = false;
    const Control sampled{mean[t].vx + config_.noise_v_mps * previous.vx,
                          mean[t].vy + config_.noise_v_mps * previous.vy,
                          mean[t].wz + config_.noise_w_radps * previous.wz};
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
  const double rho = config_.noise_correlation;
  for (std::size_t t = 0; t < mean.size(); ++t) {
    if (!active[t])
      continue;
    // Whiten both vectors with the AR(1) precision operator. A masked interval
    // starts a new active segment; inactive controls cannot affect this cost.
    const bool linked = t > 0 && active[t - 1];
    const double a = linked ? rho : 0.0;
    const double variance_scale = linked ? 1.0 - rho * rho : 1.0;
    const Control m = linked ? mean[t - 1] : Control{};
    const Control n = linked ? noise[t - 1] : Control{};
    if (config_.noise_v_mps > 0.0)
      sum += ((mean[t].vx - a * m.vx) * (noise[t].vx - a * n.vx) +
              (mean[t].vy - a * m.vy) * (noise[t].vy - a * n.vy)) /
             (config_.noise_v_mps * config_.noise_v_mps * variance_scale);
    if (config_.noise_w_radps > 0.0)
      sum += (mean[t].wz - a * m.wz) * (noise[t].wz - a * n.wz) /
             (config_.noise_w_radps * config_.noise_w_radps * variance_scale);
  }
  return config_.control_correction_weight * sum;
}
void NoiseGenerator::reset() {
  rng_.seed(config_.random_seed);
  normal_.reset();
}
} // namespace swerve_mppi
