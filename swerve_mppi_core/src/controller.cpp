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
  warm_elapsed_ = mode_age_ = 0;
  observed_ = false;
  random_.seed(config_.seed);
  normal_.reset();
}
std::vector<Velocity> Controller::seed(const Input & input, Mode mode) const
{
  State s = input.state;
  std::vector<Velocity> out;
  out.reserve(config_.horizon);
  for (std::size_t i = 0; i < config_.horizon; ++i) {
    auto target = target_point(s.pose, input.path, config_.lookahead);
    double dx = target.x - s.pose.x, dy = target.y - s.pose.y;
    double x = std::cos(s.pose.yaw) * dx + std::sin(s.pose.yaw) * dy;
    double y = -std::sin(s.pose.yaw) * dx + std::cos(s.pose.yaw) * dy;
    double limit = config_.max_speed;
    if (input.goal_active && distance(input.path.back(), input.goal) < 0.001)
      limit = std::min(limit, config_.approach_gain * distance(s.pose, input.goal));
    Velocity v;
    if (mode == Mode::Spin) {
      const double desired =
        input.goal_active && distance(s.pose, input.goal) < config_.approach_distance
          ? input.goal.yaw
          : target.yaw;
      v.wz = 1.5 * wrap_angle(desired - s.pose.yaw);
    } else if (mode == Mode::Crab) {
      v = {config_.approach_gain * x, config_.approach_gain * y, 0};
      double scale = std::max(1.0, std::hypot(v.vx, v.vy) / std::max(limit, 1e-12));
      v.vx /= scale;
      v.vy /= scale;
    } else {
      v.vx = std::clamp(config_.approach_gain * x, -limit, limit);
      if (std::abs(x) < 0.05 && std::abs(y) > 0.15) v.vx = std::min(limit, 0.25);
      v.wz = v.vx * 2 * y / std::max(0.04, x * x + y * y);
    }
    v = model_.project(v, mode);
    out.push_back(v);
    const auto next = model_.rollout(s, mode, {v});
    if (next.valid) s = next.states.back();
  }
  return out;
}
Result Controller::compute(const Input & input)
{
  const auto begin = std::chrono::steady_clock::now();
  Result result;
  result.command.mode = input.state.mode;
  auto finish = [&]() {
    result.compute_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    return result;
  };
  if (
    input.path.empty() || input.path.size() > 4096 || !finite(input.state.pose) ||
    !finite(input.state.velocity) || !finite(input.goal) || !valid_mode(input.state.mode) ||
    !valid_mode(input.state.requested_mode) || !std::isfinite(input.dt) || input.dt <= 0 ||
    input.dt > 1 ||
    std::any_of(input.path.begin(), input.path.end(), [](Pose p) { return !finite(p); })) {
    reset();
    return finish();
  }
  for (std::size_t i = 0; i < 4; ++i)
    if (
      !std::isfinite(input.state.steering[i]) || !std::isfinite(input.state.wheel_speeds[i]) ||
      std::abs(input.state.steering[i]) > config_.steer_limit + 0.05) {
      reset();
      return finish();
    }
  if (!observed_ || observed_mode_ != input.state.mode) {
    warm_.clear();
    mode_age_ = observed_ ? 0 : config_.minimum_mode_dwell;
    warm_elapsed_ = 0;
    observed_mode_ = input.state.mode;
    observed_ = true;
  }
  if (input.state.switching) {
    mode_age_ = 0;
    warm_.clear();
    warm_elapsed_ = 0;
    result.command.mode = input.state.requested_mode;
    result.status = Status::Switching;
    return finish();
  }
  mode_age_ += input.dt;
  warm_elapsed_ += input.dt;
  if (!warm_.empty()) {
    const auto shift =
      std::min(warm_.size(), static_cast<std::size_t>(warm_elapsed_ / config_.model_dt));
    if (shift) {
      warm_.erase(warm_.begin(), warm_.begin() + shift);
      warm_.resize(config_.horizon, warm_.empty() ? Velocity{} : warm_.back());
      warm_elapsed_ -= shift * config_.model_dt;
    }
  }
  CriticContext context{input, config_};
  try {
    for (auto & critic : critics_) critic->prepare(context);
  } catch (const std::exception &) {
    warm_.clear();
    result.status = Status::NoSolution;
    return finish();
  }
  const bool terminal = input.goal_active && distance(input.path.back(), input.goal) < 0.001;
  const double terminal_scale =
    terminal ? std::clamp(
                 std::max(
                   distance(input.state.pose, input.goal) / config_.approach_distance,
                   std::abs(wrap_angle(input.state.pose.yaw - input.goal.yaw)) / 0.5),
                 0.02, 1.0)
             : 1.0;
  double selected_cost = std::numeric_limits<double>::infinity(), keep_cost = selected_cost;
  std::vector<Velocity> selected;
  Trajectory selected_trace;
  Mode selected_mode = input.state.mode;
  const Mode modes[] = {input.state.mode, Mode::Ackermann, Mode::Spin, Mode::Crab};
  for (std::size_t branch = 0; branch < 4; ++branch) {
    Mode mode = modes[branch];
    if (branch && mode == input.state.mode) continue;
    if (branch && mode_age_ < config_.minimum_mode_dwell) continue;
    auto fresh = seed(input, mode);
    auto mean = mode == input.state.mode && warm_.size() == config_.horizon ? warm_ : fresh;
    double best = std::numeric_limits<double>::infinity();
    std::vector<Velocity> best_controls;
    Trajectory best_trace;
    auto score = [&](const std::vector<Trajectory> & traces) {
      std::vector<double> costs(
        traces.size(), mode == input.state.mode ? 0 : config_.switch_penalty * terminal_scale);
      for (std::size_t k = 0; k < traces.size(); ++k)
        if (!traces[k].valid) costs[k] = std::numeric_limits<double>::infinity();
      for (const auto & critic : critics_) critic->score(context, traces, costs);
      if (costs.size() != traces.size()) throw std::runtime_error("critic changed batch size");
      for (double & cost : costs)
        if (!std::isfinite(cost)) cost = std::numeric_limits<double>::infinity();
      result.evaluated += traces.size();
      return costs;
    };
    try {
      for (std::size_t iteration = 0; iteration < config_.iterations; ++iteration) {
        std::vector<Trajectory> batch;
        batch.reserve(config_.samples);
        for (std::size_t k = 0; k < config_.samples; ++k) {
          auto candidate = k == 1 ? fresh : mean;
          Velocity noise;
          if (k >= 2)
            for (std::size_t t = 0; t < candidate.size(); ++t) {
              const double a = t ? config_.noise_correlation : 0, b = t ? std::sqrt(1 - a * a) : 1;
              noise = {
                a * noise.vx + b * normal_(random_), a * noise.vy + b * normal_(random_),
                a * noise.wz + b * normal_(random_)};
              auto & u = candidate[t];
              u = model_.project(
                {u.vx + config_.noise_linear * terminal_scale * noise.vx,
                 u.vy + config_.noise_linear * terminal_scale * noise.vy,
                 u.wz + config_.noise_angular * terminal_scale * noise.wz},
                mode);
            }
          batch.push_back(model_.rollout(input.state, mode, candidate));
        }
        const auto costs = score(batch);
        const auto winner =
          static_cast<std::size_t>(std::min_element(costs.begin(), costs.end()) - costs.begin());
        if (!std::isfinite(costs[winner])) break;
        if (costs[winner] < best) {
          best = costs[winner];
          best_controls = batch[winner].controls;
          best_trace = batch[winner];
        }
        std::vector<Velocity> weighted(config_.horizon);
        double total = 0;
        for (std::size_t k = 0; k < batch.size(); ++k) {
          double w = std::exp(-(costs[k] - costs[winner]) / config_.temperature);
          total += w;
          for (std::size_t t = 0; t < weighted.size(); ++t) {
            weighted[t].vx += w * batch[k].controls[t].vx;
            weighted[t].vy += w * batch[k].controls[t].vy;
            weighted[t].wz += w * batch[k].controls[t].wz;
          }
        }
        for (auto & u : weighted)
          u = model_.project({u.vx / total, u.vy / total, u.wz / total}, mode);
        std::vector<Trajectory> update{model_.rollout(input.state, mode, weighted)};
        double cost = score(update).front();
        if (cost <= best) {
          best = cost;
          best_controls = weighted;
          best_trace = std::move(update.front());
        }
        mean = cost <= costs[winner] ? weighted : batch[winner].controls;
      }
    } catch (const std::exception &) {
      warm_.clear();
      result.status = Status::NoSolution;
      return finish();
    }
    if (branch == 0) keep_cost = best;
    if (
      best < selected_cost &&
      (mode == input.state.mode || best + config_.switch_hysteresis * terminal_scale < keep_cost)) {
      selected_cost = best;
      selected_mode = mode;
      selected = std::move(best_controls);
      selected_trace = std::move(best_trace);
    }
  }
  if (selected.empty()) {
    warm_.clear();
    result.status = Status::NoSolution;
    return finish();
  }
  result.status = Status::Ok;
  result.command = {selected.front(), selected_mode};
  result.cost = selected_cost;
  for (const auto & state : selected_trace.states) result.prediction.push_back(state.pose);
  if (selected_mode == input.state.mode)
    warm_ = std::move(selected);
  else
    warm_.clear();
  return finish();
}
}  // namespace swerve_mppi
