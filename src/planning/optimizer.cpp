#include "swerve_mppi/planning/optimizer.hpp"

#include <algorithm>
#include <cmath>

#include "safety/detail/validation.hpp"
#include "swerve_mppi/feedback/feedback.hpp"

namespace swerve_mppi
{
namespace
{
double clamp(double value, double low, double high) { return std::clamp(value, low, high); }
}  // namespace
Optimizer::Optimizer(const Config & config, std::shared_ptr<const TrajectoryValidator> validator)
: config_(config),
  model_(config),
  rollout_(config),
  critics_(config, std::move(validator)),
  noise_(config)
{
  samples_.resize(config_.samples_per_branch);
  for (auto & sample : samples_) {
    sample.noise.resize(config_.horizon_steps);
    sample.active.resize(config_.horizon_steps);
  }
  candidate_.resize(config_.horizon_steps);
  weighted_.resize(config_.horizon_steps);
}
std::vector<Control> Optimizer::seed(
  const ControllerInput & input, const Branch & branch, bool use_warm) const
{
  if (
    use_warm && !branch.switches && warm_mode_ == input.vehicle.actual_mode &&
    warm_start_.size() == config_.horizon_steps) {
    return warm_start_;
  }
  const Pose2d & goal = input.reference_path.back();
  const Pose2d & pose = input.vehicle.pose;
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
      u.vx = clamp(
        local_x / (config_.dt_s * config_.horizon_steps), -config_.max_vx_mps, config_.max_vx_mps);
      if (std::abs(u.vx) < 0.15 && std::abs(local_y) > 0.2) {
        u.vx = 0.35;
      }
      // Signed pure-pursuit curvature preserves reverse travel semantics.
      const double curvature = 2.0 * local_y / std::max(.04, local_x * local_x + local_y * local_y);
      u.wz = clamp(u.vx * curvature, -config_.max_yaw_rate_radps, config_.max_yaw_rate_radps);
    } else if (mode == DriveMode::Spin) {
      u.wz =
        clamp(angle_distance(goal.yaw, pose.yaw), -config_.max_spin_radps, config_.max_spin_radps);
    } else {
      u.vx = local_x / (config_.dt_s * config_.horizon_steps);
      u.vy = local_y / (config_.dt_s * config_.horizon_steps);
    }
    controls[i] = model_.project(u, mode);
  }
  return controls;
}

Solution Optimizer::optimize(
  const ControllerInput & input, const Branch & branch, const PlanningBudget * budget)
{
  PlanningBudget local_budget(budget ? 0 : config_.dt_s * config_.compute_budget_ratio);
  const auto & active_budget = budget ? *budget : local_budget;
  Solution result;
  result.branch = branch;
  if (active_budget.expired()) {
    result.planning_stats.budget_exhausted = true;
    clear_warm_start();
    return result;
  }
  if (
    !detail::valid_input(input, config_) ||
    check_model_feedback(input.vehicle, config_).status != FeedbackStatus::Valid) {
    return result;
  }
  // Anticipate the yaw-rate budget of the ordered local curve. A horizon with
  // limited steering cannot track a tight bend at the straight-line speed cap.
  double curvature = 0;
  Pose2d a = input.reference_path.front(), b = a;
  bool have_segment = false;
  for (std::size_t i = 1; i < input.reference_path.size(); ++i) {
    const auto & d = input.reference_path[i];
    const double bx = d.x - b.x, by = d.y - b.y;
    const double second = std::hypot(bx, by);
    if (second <= 1e-9) {
      continue;
    }
    if (have_segment) {
      const double ax = b.x - a.x, ay = b.y - a.y;
      curvature = std::max(
        curvature, std::abs(std::atan2(ax * by - ay * bx, ax * bx + ay * by)) /
                     ((std::hypot(ax, ay) + second) / 2));
    }
    a = b;
    b = d;
    have_segment = true;
  }
  const auto constrain = [&](Control u, std::size_t t) {
    const auto mode =
      branch.switches && t >= branch.switch_step ? branch.mode : input.vehicle.actual_mode;
    u = model_.project(u, mode);
    if (mode != DriveMode::Spin) {
      const double speed = std::hypot(u.vx, u.vy);
      double limit = input.tracking            ? input.tracking->speed_limit_mps
                     : mode == DriveMode::Crab ? config_.max_crab_speed_mps
                                               : config_.max_vx_mps;
      if (mode == DriveMode::DualAckermann && curvature > 1e-9) {
        limit = std::min(limit, config_.max_yaw_rate_radps / curvature);
      }
      const double scale = speed > limit ? limit / speed : 1.0;
      u = {u.vx * scale, u.vy * scale, u.wz * scale};
    }
    return model_.project(u, mode);
  };
  auto mean = seed(input, branch);
  for (std::size_t t = 0; t < mean.size(); ++t) {
    mean[t] = constrain(mean[t], t);
  }
  PlanningStats stats;
  stats.branches = 1;
  const auto timeout = [&]() {
    result.cost = std::numeric_limits<double>::infinity();
    result.controls.clear();
    result.trajectory.valid = false;
    stats.budget_exhausted = true;
    result.planning_stats = stats;
    clear_warm_start();
    return result;
  };
  auto evaluate = [&](const std::vector<Control> & controls) {
    rollout_.generate(input.vehicle, branch, controls, proposal_);
    ++stats.evaluated_rollouts;
    const double cost = critics_.score(input, proposal_);
    stats.feasible_rollouts += std::isfinite(cost);
    return cost;
  };
  auto capture = [&](Solution & out, const std::vector<Control> & controls, double cost) {
    out.branch = branch;
    out.controls = controls;
    out.trajectory = proposal_;
    out.cost = cost;
  };
  const double nominal_cost = evaluate(mean);
  if (active_budget.expired()) {
    return timeout();
  }
  capture(result, mean, nominal_cost);
  const auto fresh_seed = input.tracking ? seed(input, branch, false) : std::vector<Control>{};
  std::size_t feasible = 0;
  for (std::size_t iteration = 0; iteration < config_.iterations; ++iteration) {
    if (active_budget.expired()) {
      return timeout();
    }
    double minimum = std::numeric_limits<double>::infinity();
    Solution fallback = result;
    for (std::size_t k = 0; k < samples_.size(); ++k) {
      if (active_budget.expired()) {
        return timeout();
      }
      auto & sample = samples_[k];
      if (k == 0) {
        // Preserve the nominal proposal when noisy steering-sensitive paths
        // stall.
        candidate_ = mean;
      } else if (k == 1 && input.tracking) {
        candidate_ = fresh_seed;
      } else {
        noise_.sample(mean, branch, input.vehicle.actual_mode, candidate_, sample.noise);
      }
      for (std::size_t t = 0; t < candidate_.size(); ++t) {
        candidate_[t] = constrain(candidate_[t], t);
        sample.noise[t] = {
          candidate_[t].vx - mean[t].vx, candidate_[t].vy - mean[t].vy,
          candidate_[t].wz - mean[t].wz};
      }
      sample.cost = evaluate(candidate_);
      if (active_budget.expired()) {
        return timeout();
      }
      sample.active = proposal_.active_controls;
      if (!std::isfinite(sample.cost)) {
        continue;
      }
      ++feasible;
      if (sample.cost < fallback.cost) {
        capture(fallback, candidate_, sample.cost);
      }
      sample.cost += noise_.correction(mean, sample.noise, sample.active, branch);
      if (std::isfinite(sample.cost)) {
        minimum = std::min(minimum, sample.cost);
      }
    }
    if (!std::isfinite(minimum)) {
      result = std::move(fallback);
      ++stats.fallback_updates;
      break;
    }
    std::fill(weighted_.begin(), weighted_.end(), Control{});
    double total = 0.0;
    for (const auto & sample : samples_) {
      if (!std::isfinite(sample.cost)) {
        continue;
      }
      const double weight = std::exp(-(sample.cost - minimum) / config_.temperature);
      total += weight;
      for (std::size_t t = 0; t < mean.size(); ++t) {
        if (!sample.active[t]) {
          continue;
        }
        weighted_[t].vx += weight * sample.noise[t].vx;
        weighted_[t].vy += weight * sample.noise[t].vy;
        weighted_[t].wz += weight * sample.noise[t].wz;
      }
    }
    if (!(total > 0.0)) {
      result = std::move(fallback);
      ++stats.fallback_updates;
      break;
    }
    for (std::size_t t = 0; t < mean.size(); ++t) {
      mean[t].vx += weighted_[t].vx / total;
      mean[t].vy += weighted_[t].vy / total;
      mean[t].wz += weighted_[t].wz / total;
      mean[t] = constrain(mean[t], t);
    }
    // Return the weighted MPPI sequence; a feasible proposal is only a
    // fallback.
    const double updated_cost = evaluate(mean);
    if (active_budget.expired()) {
      return timeout();
    }
    if (std::isfinite(updated_cost)) {
      capture(result, mean, updated_cost);
    } else {
      result = std::move(fallback);
      mean = result.controls;
      ++stats.fallback_updates;
    }
  }
  result.planning_stats = stats;
  result.feasible_rollouts = feasible;
  return result;
}
void Optimizer::accept(const Solution & solution, DriveMode mode)
{
  if (
    solution.branch.switches || !std::isfinite(solution.cost) ||
    solution.controls.size() != config_.horizon_steps) {
    warm_start_.clear();
    return;
  }
  warm_mode_ = mode;
  warm_start_ = solution.controls;
  std::rotate(warm_start_.begin(), warm_start_.begin() + 1, warm_start_.end());
  warm_start_.back() = solution.controls.back();
}
void Optimizer::clear_warm_start() { warm_start_.clear(); }
void Optimizer::reset()
{
  clear_warm_start();
  noise_.reset();
}
}  // namespace swerve_mppi
