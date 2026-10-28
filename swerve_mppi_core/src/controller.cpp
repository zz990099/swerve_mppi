#include "swerve_mppi/controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/path.hpp"

namespace swerve_mppi
{
namespace
{
bool finite(Velocity v)
{
  return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.wz);
}
bool finite(Pose p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw); }
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
  PathView path(input.path);
  double progress = path.match(s.pose).arc;
  std::vector<Velocity> out;
  out.reserve(config_.horizon);
  for (std::size_t i = 0; i < config_.horizon; ++i) {
    auto m = path.match(
      s.pose, progress,
      std::min(path.length(), progress + config_.max_speed * config_.model_dt * 1.5 + .02));
    progress = m.arc;
    double corner = path.next_corner(progress);
    if (
      corner < path.length() - 1e-5 && corner - progress < .015 &&
      std::hypot(s.velocity.vx, s.velocity.vy) < .04) {
      progress = corner;
      m = path.match(s.pose, progress, progress + .05);
    }
    const double servo_rate = std::min(
      config_.limits.steering_rate * .6,
      config_.limits.tracking_tolerance / (1.4 * config_.steering_time_constant));
    double speed = path.speed_limit(
      progress, config_.max_speed, config_.limits.linear_acceleration, config_.approach_gain,
      servo_rate);
    double wx = speed * m.tx - config_.cross_gain * (s.pose.x - m.pose.x);
    double wy = speed * m.ty - config_.cross_gain * (s.pose.y - m.pose.y);
    double wz = path.yaw_derivative(progress) * speed + 2.5 * wrap_angle(m.pose.yaw - s.pose.yaw);
    Velocity desired{
      std::cos(s.pose.yaw) * wx + std::sin(s.pose.yaw) * wy,
      -std::sin(s.pose.yaw) * wx + std::cos(s.pose.yaw) * wy, wz};
    auto v = model_.condition(
      desired, s.requested, s.generator.command.steering, i ? config_.model_dt : input.dt);
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
    auto shift = warm_elapsed_ >= warm_.size() * config_.model_dt
                   ? warm_.size()
                   : static_cast<std::size_t>(warm_elapsed_ / config_.model_dt);
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
    auto condition = [&](std::vector<Velocity> controls) {
      auto previous = input.state.requested;
      auto reference = input.state.generator.command.steering;
      for (std::size_t i = 0; i < controls.size(); ++i) {
        controls[i] =
          model_.condition(controls[i], previous, reference, i ? config_.model_dt : input.dt);
        auto wheels = swerve_chassis_core::inverse(
          config_.geometry, {controls[i].vx, controls[i].vy, controls[i].wz}, reference);
        reference = wheels.steering;
        previous = controls[i];
      }
      return controls;
    };
    auto intent = swerve_chassis_core::inverse(
      config_.geometry, {fresh[0].vx, fresh[0].vy, fresh[0].wz},
      input.state.generator.command.steering);
    double alignment = 1;
    for (std::size_t i = 0; i < 4; ++i)
      alignment =
        std::min(alignment, std::max(0., std::cos(intent.steering[i] - input.state.steering[i])));
    alignment *= alignment;
    bool rotation_only = PathView(input.path).length() < 1e-6;
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
          for (auto & u : candidate) {
            u.vx *= .7;
            u.vy *= .7;
            u.wz *= .7;
          }
        Velocity noise;
        if (k >= 4)
          for (auto & u : candidate) {
            const double a = config_.noise_correlation, b = std::sqrt(1 - a * a);
            noise = {
              a * noise.vx + b * normal_(random_), a * noise.vy + b * normal_(random_),
              a * noise.wz + b * normal_(random_)};
            u = model_.project(
              {u.vx +
                 (rotation_only ? 0 : config_.noise_linear) * noise_scale * alignment * noise.vx,
               u.vy +
                 (rotation_only ? 0 : config_.noise_linear) * noise_scale * alignment * noise.vy,
               u.wz + config_.noise_angular * noise_scale * alignment * noise.wz});
          }
        batch.push_back(model_.rollout(input.state, condition(std::move(candidate))));
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
      double total = 0, squared = 0;
      for (std::size_t k = 0; k < batch.size(); ++k) {
        const double w = std::exp(-(costs[k] - costs[winner]) / config_.temperature);
        total += w;
        squared += w * w;
        for (std::size_t t = 0; t < weighted.size(); ++t) {
          weighted[t].vx += w * batch[k].controls[t].vx;
          weighted[t].vy += w * batch[k].controls[t].vy;
          weighted[t].wz += w * batch[k].controls[t].wz;
        }
      }
      result.effective_samples = total * total / squared;
      for (auto & u : weighted) u = model_.project({u.vx / total, u.vy / total, u.wz / total});
      weighted = condition(std::move(weighted));
      std::vector<Trajectory> update{model_.rollout(input.state, weighted)};
      const double cost = score(update).front();
      if (cost <= best + .02) {
        best = cost;
        selected = weighted;
        selected_trace = std::move(update.front());
      }
      mean = cost <= costs[winner] + .02 ? weighted : batch[winner].controls;
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
