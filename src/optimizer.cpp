#include "swerve_mppi/optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace swerve_mppi {
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double path_distance(const Pose2d & pose, const std::vector<Pose2d> & path) {
  double nearest = kInfinity;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    const double length2 = dx * dx + dy * dy;
    const double t = length2 > 1e-12 ?
        clamp(((pose.x - path[i - 1].x) * dx +
               (pose.y - path[i - 1].y) * dy) / length2, 0.0, 1.0) : 0.0;
    nearest = std::min(nearest, std::hypot(
        pose.x - (path[i - 1].x + t * dx),
        pose.y - (path[i - 1].y + t * dy)));
  }
  return path.size() == 1 ?
      std::hypot(pose.x - path[0].x, pose.y - path[0].y) : nearest;
}

double obstacle_cost(const Pose2d & previous, const Pose2d & next,
                     const ControllerInput & input, const Config & config) {
  double cost = 0.0;
  const double dx = next.x - previous.x;
  const double dy = next.y - previous.y;
  const double length2 = dx * dx + dy * dy;
  for (const auto & obstacle : input.obstacles) {
    const double t = length2 > 1e-12 ?
        clamp(((obstacle.x - previous.x) * dx +
               (obstacle.y - previous.y) * dy) / length2, 0.0, 1.0) : 0.0;
    const double clearance = std::hypot(previous.x + t * dx - obstacle.x,
                                        previous.y + t * dy - obstacle.y) -
        obstacle.radius - config.robot_radius_m - config.collision_margin_m;
    if (clearance <= 0.0) return kInfinity;
    if (clearance < 0.5) {
      cost += config.clearance_weight / (clearance + 0.03);
    }
  }
  return cost;
}

double sample_pose_cost(const Pose2d & previous, const Pose2d & next,
                        const ControllerInput & input, const Config & config) {
  const double obstacle = obstacle_cost(previous, next, input, config);
  if (!std::isfinite(obstacle)) return kInfinity;
  const double d = path_distance(next, input.reference_path);
  return config.dt_s * (config.path_weight * d * d + obstacle);
}

bool same_branch(const Branch & a, const Branch & b) {
  return a.mode == b.mode && a.switches == b.switches &&
         a.switch_step == b.switch_step;
}
}  // namespace

Optimizer::Optimizer(const Config & config)
    : config_(config), model_(config_), transition_(config_, model_),
      rng_(config.random_seed) {
  validate(config_);
}

std::vector<Branch> Optimizer::make_branches(const VehicleState & state) const {
  std::vector<Branch> branches{{state.actual_mode, 0, false}};
  const double wait = config_.minimum_mode_dwell_s - state.time_in_mode_s;
  if (!state.mode_confirmed || state.mode_fault) return branches;
  const std::size_t earliest = wait <= 0.0 ? 0 :
      static_cast<std::size_t>(std::ceil(wait / config_.dt_s));
  const DriveMode modes[] = {DriveMode::DualAckermann, DriveMode::Spin,
                             DriveMode::Crab};
  for (DriveMode mode : modes) {
    if (mode == state.actual_mode) continue;
    if (earliest < config_.horizon_steps / 2) {
      branches.push_back({mode, earliest, true});
    }
    const std::size_t later = std::max(earliest, config_.horizon_steps / 3);
    if (later != earliest && later < config_.horizon_steps / 2) {
      branches.push_back({mode, later, true});
    }
  }
  return branches;
}

std::vector<Control> Optimizer::seed(const ControllerInput & input,
                                     const Branch & branch) const {
  for (const auto & entry : warm_starts_) {
    if (same_branch(entry.first, branch) &&
        entry.second.size() == config_.horizon_steps) return entry.second;
  }
  const Pose2d & goal = input.reference_path.back();
  const Pose2d & pose = input.vehicle.pose;
  const double dx = goal.x - pose.x;
  const double dy = goal.y - pose.y;
  const double local_x = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
  const double local_y = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  std::vector<Control> controls(config_.horizon_steps);
  for (std::size_t i = 0; i < controls.size(); ++i) {
    const DriveMode mode = branch.switches && i >= branch.switch_step ?
        branch.mode : input.vehicle.actual_mode;
    Control u;
    if (mode == DriveMode::DualAckermann) {
      u.vx = clamp(local_x / (config_.dt_s * config_.horizon_steps),
                   -config_.max_vx_mps, config_.max_vx_mps);
      if (std::abs(u.vx) < 0.15 && std::abs(local_y) > 0.2) u.vx = 0.35;
      const double heading = std::atan2(local_y, std::max(0.2, local_x));
      u.wz = clamp(1.5 * heading, -config_.max_yaw_rate_radps,
                   config_.max_yaw_rate_radps);
    } else if (mode == DriveMode::Spin) {
      u.wz = clamp(angle_distance(goal.yaw, pose.yaw),
                   -config_.max_spin_radps, config_.max_spin_radps);
    } else {
      u.vx = local_x / (config_.dt_s * config_.horizon_steps);
      u.vy = local_y / (config_.dt_s * config_.horizon_steps);
    }
    controls[i] = model_.project(u, mode);
  }
  return controls;
}

double Optimizer::score(const ControllerInput & input, const Branch & branch,
                        const std::vector<Control> & controls) const {
  VehicleState state = input.vehicle;
  if (!std::isfinite(obstacle_cost(state.pose, state.pose, input, config_))) {
    return kInfinity;
  }
  double cost = branch.switches ? config_.switch_cost : 0.0;
  std::size_t step = 0;
  while (step < config_.horizon_steps) {
    if (branch.switches && step == branch.switch_step) {
      std::vector<Pose2d> trace;
      const Pose2d before = state.pose;
      if (transition_.rollout(state, branch.mode, step,
                              config_.horizon_steps, &trace) < 0.0) return kInfinity;
      Pose2d previous = before;
      for (const auto & next : trace) {
        const double part = sample_pose_cost(previous, next, input, config_);
        if (!std::isfinite(part)) return kInfinity;
        cost += part;
        previous = next;
      }
      continue;
    }
    const DriveMode mode = branch.switches && step > branch.switch_step ?
        branch.mode : input.vehicle.actual_mode;
    const Control u = model_.project(controls[step], mode);
    const Pose2d before = state.pose;
    const StepResult result = model_.step(state, u, config_.dt_s);
    if (!result.valid) return kInfinity;
    state = result.state;
    const double part = sample_pose_cost(before, state.pose, input, config_);
    if (!std::isfinite(part)) return kInfinity;
    cost += part + config_.effort_weight * config_.dt_s *
             (u.vx * u.vx + u.vy * u.vy + u.wz * u.wz);
    ++step;
  }
  const Pose2d & goal = input.reference_path.back();
  const double d = std::hypot(state.pose.x - goal.x, state.pose.y - goal.y);
  const double yaw = angle_distance(state.pose.yaw, goal.yaw);
  return cost + config_.goal_weight * d * d + config_.yaw_weight * yaw * yaw;
}

Solution Optimizer::optimize(const ControllerInput & input,
                             const Branch & branch) {
  Solution result;
  result.branch = branch;
  std::vector<Control> mean = seed(input, branch);
  result.controls = mean;
  result.cost = score(input, branch, mean);
  std::normal_distribution<double> unit_noise(0.0, 1.0);
  for (std::size_t iteration = 0; iteration < config_.iterations; ++iteration) {
    struct Sample {
      std::vector<Control> perturbations;
      double cost;
    };
    std::vector<Sample> samples;
    samples.reserve(config_.samples_per_branch);
    double minimum = kInfinity;
    for (std::size_t k = 0; k < config_.samples_per_branch; ++k) {
      auto candidate = mean;
      Sample sample;
      sample.perturbations.resize(candidate.size());
      for (std::size_t t = 0; t < candidate.size(); ++t) {
        Control noise{config_.noise_v_mps * unit_noise(rng_),
                      config_.noise_v_mps * unit_noise(rng_),
                      config_.noise_w_radps * unit_noise(rng_)};
        sample.perturbations[t] = noise;
        candidate[t].vx += noise.vx;
        candidate[t].vy += noise.vy;
        candidate[t].wz += noise.wz;
        const DriveMode mode = branch.switches && t >= branch.switch_step ?
            branch.mode : input.vehicle.actual_mode;
        candidate[t] = model_.project(candidate[t], mode);
      }
      sample.cost = score(input, branch, candidate);
      if (std::isfinite(sample.cost)) {
        ++result.feasible_rollouts;
        minimum = std::min(minimum, sample.cost);
        if (sample.cost < result.cost) {
          result.cost = sample.cost;
          result.controls = candidate;
        }
      }
      samples.push_back(std::move(sample));
    }
    if (!std::isfinite(minimum)) break;
    std::vector<Control> weighted(mean.size());
    double total_weight = 0.0;
    for (const auto & sample : samples) {
      if (!std::isfinite(sample.cost)) continue;
      const double weight = std::exp(-(sample.cost - minimum) /
                                      config_.temperature);
      total_weight += weight;
      for (std::size_t t = 0; t < mean.size(); ++t) {
        weighted[t].vx += weight * sample.perturbations[t].vx;
        weighted[t].vy += weight * sample.perturbations[t].vy;
        weighted[t].wz += weight * sample.perturbations[t].wz;
      }
    }
    if (total_weight <= 0.0) break;
    for (std::size_t t = 0; t < mean.size(); ++t) {
      mean[t].vx += weighted[t].vx / total_weight;
      mean[t].vy += weighted[t].vy / total_weight;
      mean[t].wz += weighted[t].wz / total_weight;
      const DriveMode mode = branch.switches && t >= branch.switch_step ?
          branch.mode : input.vehicle.actual_mode;
      mean[t] = model_.project(mean[t], mode);
    }
    const double updated_cost = score(input, branch, mean);
    if (updated_cost < result.cost) {
      result.cost = updated_cost;
      result.controls = mean;
    }
  }
  std::vector<Control> shifted = result.controls;
  if (!shifted.empty()) {
    std::rotate(shifted.begin(), shifted.begin() + 1, shifted.end());
    shifted.back() = result.controls.back();
  }
  auto found = std::find_if(warm_starts_.begin(), warm_starts_.end(),
      [&](const auto & entry) { return same_branch(entry.first, branch); });
  if (found != warm_starts_.end()) found->second = std::move(shifted);
  else warm_starts_.push_back({branch, std::move(shifted)});
  return result;
}

void Optimizer::reset() { warm_starts_.clear(); }

}  // namespace swerve_mppi
