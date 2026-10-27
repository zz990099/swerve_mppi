#include "swerve_mppi/controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_mppi
{
namespace
{
bool finite(Velocity v)
{
  return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.wz);
}
bool finite(Pose p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw); }
Pose target_point(const Pose & pose, const std::vector<Pose> & path, double lookahead)
{
  if (path.size() == 1) return path.front();
  double best = std::numeric_limits<double>::infinity(), fraction = 0;
  std::size_t segment = 1;
  for (std::size_t i = 1; i < path.size(); ++i) {
    double dx = path[i].x - path[i - 1].x, dy = path[i].y - path[i - 1].y, l2 = dx * dx + dy * dy;
    double t =
      l2 > 1e-12
        ? std::clamp(((pose.x - path[i - 1].x) * dx + (pose.y - path[i - 1].y) * dy) / l2, 0.0, 1.0)
        : 0;
    double e = std::hypot(pose.x - path[i - 1].x - t * dx, pose.y - path[i - 1].y - t * dy);
    if (e < best) {
      best = e;
      fraction = t;
      segment = i;
    }
  }
  for (std::size_t i = segment; i < path.size(); ++i) {
    const auto & a = path[i - 1];
    const auto & b = path[i];
    double length = distance(a, b);
    double available = length * (1 - fraction);
    if (length > 1e-12 && lookahead < available) {
      double t = fraction + lookahead / length;
      return {
        a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
        wrap_angle(a.yaw + t * wrap_angle(b.yaw - a.yaw))};
    }
    lookahead -= available;
    fraction = 0;
  }
  return path.back();
}
}  // namespace
Controller::Controller(Config config, Critics critics)
: config_(config), critics_(std::move(critics)), model_(config), random_(config.seed)
{
  if (critics_.empty() || std::any_of(critics_.begin(), critics_.end(), [](const auto & p) {
        return !p;
      }))
    throw std::invalid_argument("at least one non-null critic is required");
}
void Controller::reset()
{
  warm_.clear();
  warm_elapsed_ = 0;
  random_.seed(config_.seed);
  normal_.reset();
}
std::vector<Velocity> Controller::seed(const Input & input) const
{
  State s = input.state;
  std::vector<Velocity> out;
  out.reserve(config_.horizon);
  for (std::size_t i = 0; i < config_.horizon; ++i) {
    auto target = target_point(s.pose, input.path, config_.lookahead);
    double dx = target.x - s.pose.x, dy = target.y - s.pose.y;
    double limit = config_.max_speed;
    const bool terminal = input.goal_active && distance(input.path.back(), input.goal) < 0.001;
    if (terminal) limit = std::min(limit, config_.approach_gain * distance(s.pose, input.goal));
    const double yaw = terminal && distance(s.pose, input.goal) < config_.approach_distance
                         ? input.goal.yaw
                         : target.yaw;
    Velocity v{
      config_.approach_gain * (std::cos(s.pose.yaw) * dx + std::sin(s.pose.yaw) * dy),
      config_.approach_gain * (-std::sin(s.pose.yaw) * dx + std::cos(s.pose.yaw) * dy),
      1.5 * wrap_angle(yaw - s.pose.yaw)};
    double scale = std::max(1.0, std::hypot(v.vx, v.vy) / std::max(limit, 1e-12));
    v.vx /= scale;
    v.vy /= scale;
    v = model_.project(v);
    out.push_back(v);
    auto next = model_.rollout(s, {v});
    if (next.valid) s = next.states.back();
  }
  return out;
}
Result Controller::compute(const Input & input)
{
  const auto begin = std::chrono::steady_clock::now();
  Result result;
  auto finish = [&]() {
    result.compute_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    return result;
  };
  if (
    input.path.empty() || input.path.size() > 4096 || !model_.valid(input.state) ||
    !finite(input.state.velocity) || !finite(input.goal) || !std::isfinite(input.dt) ||
    input.dt <= 0 || input.dt > 1 ||
    std::any_of(input.path.begin(), input.path.end(), [](Pose p) { return !finite(p); })) {
    reset();
    return finish();
  }
  warm_elapsed_ += input.dt;
  if (!warm_.empty()) {
    auto shift = std::min(warm_.size(), static_cast<std::size_t>(warm_elapsed_ / config_.model_dt));
    if (shift) {
      warm_.erase(warm_.begin(), warm_.begin() + shift);
      warm_.resize(config_.horizon, warm_.empty() ? Velocity{} : warm_.back());
      warm_elapsed_ -= shift * config_.model_dt;
    }
  } else {
    warm_elapsed_ = 0;
  }
  CriticContext context{input, config_};
  const bool terminal = input.goal_active && distance(input.path.back(), input.goal) < 0.001;
  const double noise_scale =
    terminal ? std::clamp(
                 std::max(
                   distance(input.state.pose, input.goal) / config_.approach_distance,
                   std::abs(wrap_angle(input.state.pose.yaw - input.goal.yaw)) / 0.5),
                 0.02, 1.0)
             : 1.0;
  double best = std::numeric_limits<double>::infinity();
  std::vector<Velocity> selected;
  Trajectory selected_trace;
  try {
    for (auto & critic : critics_) critic->prepare(context);
    auto fresh = seed(input);
    auto mean = warm_.size() == config_.horizon ? warm_ : fresh;
    auto score = [&](const std::vector<Trajectory> & traces) {
      std::vector<double> costs(traces.size(), 0);
      for (const auto & critic : critics_) {
        critic->score(context, traces, costs);
        if (costs.size() != traces.size()) throw std::runtime_error("critic changed batch size");
      }
      for (std::size_t i = 0; i < traces.size(); ++i)
        if (!traces[i].valid || !std::isfinite(costs[i]))
          costs[i] = std::numeric_limits<double>::infinity();
      result.evaluated += traces.size();
      return costs;
    };
    for (std::size_t iteration = 0; iteration < config_.iterations; ++iteration) {
      std::vector<Trajectory> batch;
      batch.reserve(config_.samples);
      for (std::size_t k = 0; k < config_.samples; ++k) {
        auto candidate = k == 1 ? fresh : mean;
        if (k == 2) candidate.assign(config_.horizon, {});
        if (k == 3)
          candidate.assign(config_.horizon, fresh.front());  // Sustained presteering intent.
        Velocity noise;
        if (k >= 4)
          for (auto & u : candidate) {
            const double a = config_.noise_correlation, b = std::sqrt(1 - a * a);
            noise = {
              a * noise.vx + b * normal_(random_), a * noise.vy + b * normal_(random_),
              a * noise.wz + b * normal_(random_)};
            u = model_.project(
              {u.vx + config_.noise_linear * noise_scale * noise.vx,
               u.vy + config_.noise_linear * noise_scale * noise.vy,
               u.wz + config_.noise_angular * noise_scale * noise.wz});
          }
        batch.push_back(model_.rollout(input.state, candidate));
      }
      const auto costs = score(batch);
      const auto winner =
        static_cast<std::size_t>(std::min_element(costs.begin(), costs.end()) - costs.begin());
      if (!std::isfinite(costs[winner])) break;
      if (costs[winner] < best) {
        best = costs[winner];
        selected = batch[winner].controls;
        selected_trace = batch[winner];
      }
      std::vector<Velocity> weighted(config_.horizon);
      double total = 0;
      for (std::size_t k = 0; k < batch.size(); ++k) {
        const double w = std::exp(-(costs[k] - costs[winner]) / config_.temperature);
        total += w;
        for (std::size_t t = 0; t < weighted.size(); ++t) {
          weighted[t].vx += w * batch[k].controls[t].vx;
          weighted[t].vy += w * batch[k].controls[t].vy;
          weighted[t].wz += w * batch[k].controls[t].wz;
        }
      }
      for (auto & u : weighted) u = model_.project({u.vx / total, u.vy / total, u.wz / total});
      std::vector<Trajectory> update{model_.rollout(input.state, weighted)};
      const double cost = score(update).front();
      if (cost <= best) {
        best = cost;
        selected = weighted;
        selected_trace = std::move(update.front());
      }
      mean = cost <= costs[winner] ? weighted : batch[winner].controls;
    }
  } catch (const std::exception &) {
    reset();
    result.status = Status::NoSolution;
    return finish();
  }
  if (selected.empty()) {
    reset();
    result.status = Status::NoSolution;
    return finish();
  }
  result.status = Status::Ok;
  result.command.velocity = selected.front();
  result.cost = best;
  for (const auto & s : selected_trace.states) result.prediction.push_back(s.pose);
  warm_ = std::move(selected);
  return finish();
}
}  // namespace swerve_mppi
