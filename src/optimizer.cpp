#include "swerve_mppi/optimizer.hpp"

#include "validation.hpp"
#include <algorithm>
#include <cmath>

namespace swerve_mppi {
namespace {
double clamp(double value, double low, double high) { return std::clamp(value, low, high); }
} // namespace
Optimizer::Optimizer(const Config &config)
    : config_(config), model_(config), rollout_(config), critics_(config), noise_(config) {}
std::vector<Control> Optimizer::seed(const ControllerInput &input, const Branch &branch) const {
  if (!branch.switches && warm_mode_ == input.vehicle.actual_mode &&
      warm_start_.size() == config_.horizon_steps)
    return warm_start_;
  const Pose2d &goal = input.reference_path.back();
  const Pose2d &pose = input.vehicle.pose;
  const double dx = goal.x - pose.x;
  const double dy = goal.y - pose.y;
  const double local_x = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
  const double local_y = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  std::vector<Control> controls(config_.horizon_steps);
  for (std::size_t i = 0; i < controls.size(); ++i) {
    const DriveMode mode =
        branch.switches && i >= branch.switch_step ? branch.mode : input.vehicle.actual_mode;
    Control u;
    if (mode == DriveMode::DualAckermann) {
      u.vx = clamp(local_x / (config_.dt_s * config_.horizon_steps), -config_.max_vx_mps,
                   config_.max_vx_mps);
      if (std::abs(u.vx) < 0.15 && std::abs(local_y) > 0.2)
        u.vx = 0.35;
      const double heading = std::atan2(local_y, std::max(0.2, local_x));
      u.wz = clamp(1.5 * heading, -config_.max_yaw_rate_radps, config_.max_yaw_rate_radps);
    } else if (mode == DriveMode::Spin) {
      u.wz = clamp(angle_distance(goal.yaw, pose.yaw), -config_.max_spin_radps,
                   config_.max_spin_radps);
    } else {
      u.vx = local_x / (config_.dt_s * config_.horizon_steps);
      u.vy = local_y / (config_.dt_s * config_.horizon_steps);
    }
    controls[i] = model_.project(u, mode);
  }
  return controls;
}

Solution Optimizer::optimize(const ControllerInput &input, const Branch &branch) {
  Solution result;
  result.branch = branch;
  if (!detail::valid_input(input, config_))
    return result;
  auto mean = seed(input, branch);
  auto evaluate = [&](const std::vector<Control> &controls) {
    Solution out;
    out.branch = branch;
    out.controls = controls;
    out.trajectory = rollout_.generate(input.vehicle, branch, controls);
    out.cost = critics_.score(input, out.trajectory);
    return out;
  };
  result = evaluate(mean);
  struct Sample {
    std::vector<Control> noise;
    std::vector<bool> active;
    double cost = std::numeric_limits<double>::infinity();
  };
  std::vector<Sample> samples(config_.samples_per_branch);
  std::vector<Control> candidate;
  std::size_t feasible = 0;
  for (std::size_t iteration = 0; iteration < config_.iterations; ++iteration) {
    double minimum = std::numeric_limits<double>::infinity();
    Solution fallback = result;
    for (std::size_t k = 0; k < samples.size(); ++k) {
      auto &sample = samples[k];
      if (k == 0) {
        // Preserve the nominal proposal when steering-sensitive noisy rollouts stall.
        candidate = mean;
        sample.noise.assign(mean.size(), Control{});
      } else {
        noise_.sample(mean, branch, input.vehicle.actual_mode, candidate, sample.noise);
      }
      auto proposal = evaluate(candidate);
      sample.active = proposal.trajectory.active_controls;
      sample.cost = proposal.cost;
      if (!std::isfinite(sample.cost))
        continue;
      ++feasible;
      if (proposal.cost < fallback.cost)
        fallback = proposal;
      sample.cost += noise_.correction(mean, sample.noise, sample.active);
      if (std::isfinite(sample.cost))
        minimum = std::min(minimum, sample.cost);
    }
    if (!std::isfinite(minimum)) {
      result = std::move(fallback);
      break;
    }
    std::vector<Control> weighted(mean.size());
    double total = 0.0;
    for (const auto &sample : samples) {
      if (!std::isfinite(sample.cost))
        continue;
      const double weight = std::exp(-(sample.cost - minimum) / config_.temperature);
      total += weight;
      for (std::size_t t = 0; t < mean.size(); ++t) {
        if (!sample.active[t])
          continue;
        weighted[t].vx += weight * sample.noise[t].vx;
        weighted[t].vy += weight * sample.noise[t].vy;
        weighted[t].wz += weight * sample.noise[t].wz;
      }
    }
    if (!(total > 0.0)) {
      result = std::move(fallback);
      break;
    }
    for (std::size_t t = 0; t < mean.size(); ++t) {
      mean[t].vx += weighted[t].vx / total;
      mean[t].vy += weighted[t].vy / total;
      mean[t].wz += weighted[t].wz / total;
      const DriveMode mode =
          branch.switches && t >= branch.switch_step ? branch.mode : input.vehicle.actual_mode;
      mean[t] = model_.project(mean[t], mode);
    }
    auto updated = evaluate(mean);
    // Return the MPPI weighted sequence; a feasible sample is only a safety fallback.
    const bool valid_update = std::isfinite(updated.cost);
    result = valid_update ? std::move(updated) : std::move(fallback);
    if (!valid_update)
      mean = result.controls;
  }
  result.feasible_rollouts = feasible;
  return result;
}
void Optimizer::accept(const Solution &solution, DriveMode mode) {
  if (solution.branch.switches || !std::isfinite(solution.cost) ||
      solution.controls.size() != config_.horizon_steps) {
    warm_start_.clear();
    return;
  }
  warm_mode_ = mode;
  warm_start_ = solution.controls;
  std::rotate(warm_start_.begin(), warm_start_.begin() + 1, warm_start_.end());
  warm_start_.back() = solution.controls.back();
}
void Optimizer::reset() {
  warm_start_.clear();
  noise_.reset();
}
} // namespace swerve_mppi
