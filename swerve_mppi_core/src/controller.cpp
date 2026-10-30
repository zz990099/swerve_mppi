#include "swerve_mppi/controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/path.hpp"
#include "swerve_mppi/reference.hpp"

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
void Controller::clear_optimization()
{
  warm_.clear();
  warm_elapsed_ = 0;
  reorientation_.reset();
  random_.seed(config_.seed);
  normal_.reset();
}
void Controller::reset()
{
  clear_optimization();
  branch_.reset();
}
std::vector<Velocity> Controller::seed(
  const Input & input, const ReferenceProfile & reference,
  const std::optional<Velocity> & intent) const
{
  State s = input.state;
  PathView path(input.path);
  double progress = path.start_match(s.pose).arc;
  std::vector<Velocity> out;
  out.reserve(config_.horizon);
  bool initial_intent = true;
  for (std::size_t i = 0; i < config_.horizon; ++i) {
    auto m = path.match_ordered(
      s.pose, progress,
      std::min(path.length(), progress + config_.max_speed * config_.model_dt * 1.5 + .02));
    progress = m.arc;
    double corner = path.next_corner(progress);
    if (
      corner < path.length() - 1e-5 && distance(s.pose, path.at(corner)) < .005 &&
      std::hypot(s.velocity.vx, s.velocity.vy) < .04) {
      progress = corner;
      m = path.match(s.pose, progress, progress + .05);
    }
    const double speed = reference.speed(progress);
    const auto unit = reference.unit(progress, s.pose.yaw);
    const double ex = config_.cross_gain * (m.pose.x - s.pose.x),
                 ey = config_.cross_gain * (m.pose.y - s.pose.y);
    Velocity desired{
      speed * unit.vx + std::cos(s.pose.yaw) * ex + std::sin(s.pose.yaw) * ey,
      speed * unit.vy - std::sin(s.pose.yaw) * ex + std::cos(s.pose.yaw) * ey,
      speed * unit.wz + 2.5 * wrap_angle(m.pose.yaw - s.pose.yaw)};
    // While the wheels are stopped and reorienting, hold nominal steering
    // intent. Pose noise must not continually move the presteering target.
    bool stopped = true;
    for (double v : s.wheel_speeds) stopped &= std::abs(v) < config_.limits.stopped_speed;
    if (
      stopped && speed > .01 && m.error < .02 &&
      std::abs(wrap_angle(m.pose.yaw - s.pose.yaw)) < .06) {
      const auto nominal = reference.unit(progress, m.pose.yaw);
      // Retain the nominal direction through the instant driving starts too.
      // Millimetre position correction at +/-pi/2 can otherwise request the
      // opposite steering representation immediately after alignment.
      desired = {speed * nominal.vx, speed * nominal.vy, speed * nominal.wz};
    }
    if (!stopped) initial_intent = false;
    if (intent && stopped && initial_intent) {
      const auto target = swerve_chassis_core::inverse(
        config_.geometry, {intent->vx, intent->vy, intent->wz}, s.generator.command.steering);
      bool aligning = false;
      for (std::size_t w = 0; w < 4; ++w)
        aligning |=
          std::abs(target.steering[w] - s.steering[w]) > .8 * config_.limits.tracking_tolerance;
      if (aligning) desired = *intent;
    }
    auto v = model_.condition(
      desired, s.requested, s.generator.command.steering, i ? config_.model_dt : input.dt);
    out.push_back(v);
    auto next = model_.rollout(s, {v});
    if (next.valid) s = next.states.back();
  }
  return out;
}
Result Controller::compute(const Input & source)
{
  const auto begin = std::chrono::steady_clock::now();
  Input input = source;
  bool stopped = true;
  for (double v : source.state.wheel_speeds) stopped &= std::abs(v) < config_.limits.stopped_speed;
  // Branch events are short, ordered maneuvers in the same continuous twist
  // space. Their location and exit are held until measured completion.
  if (
    !branch_ && stopped && !source.path.empty() && source.path.size() <= 4096 &&
    model_.valid(source.state) &&
    std::all_of(source.path.begin(), source.path.end(), [](Pose p) { return finite(p); })) {
    try {
      PathView path(source.path);
      ReferenceProfile reference(source.path, config_);
      const auto match = path.start_match(source.state.pose);
      const double boundary = reference.next_branch(match.arc);
      if (
        boundary >= 0 && boundary < .10 && match.error < 2 * config_.path_corridor &&
        reference.speed(match.arc) > .01) {
        const double departure =
          std::min(path.length(), match.arc + boundary + std::max(.08, boundary + .04));
        const Pose exit = path.at(departure);
        const double dx = exit.x - source.state.pose.x, dy = exit.y - source.state.pose.y;
        const double length = std::hypot(dx, dy);
        Velocity intent{
          std::cos(exit.yaw) * dx + std::sin(exit.yaw) * dy,
          -std::sin(exit.yaw) * dx + std::cos(exit.yaw) * dy, 0};
        if (length > 1e-4) {
          intent.vx /= length;
          intent.vy /= length;
        }
        const auto target = swerve_chassis_core::inverse(
          config_.geometry, {intent.vx, intent.vy, 0}, source.state.steering);
        bool crossing = false;
        for (std::size_t w = 0; w < 4; ++w)
          crossing |=
            std::abs(target.steering[w] - source.state.steering[w]) > swerve_chassis_core::pi / 2;
        bool corridor = true;
        for (int i = 1; i <= 10; ++i) {
          const double u = i / 10.;
          Pose point{source.state.pose.x + u * dx, source.state.pose.y + u * dy, exit.yaw};
          corridor &=
            path.match(point, match.arc, departure).error < std::max(.025, match.error + .005);
        }
        if (
          crossing && corridor && length > .02 && path.next_corner(match.arc) >= departure &&
          std::abs(wrap_angle(exit.yaw - source.state.pose.yaw)) < config_.heading_corridor) {
          Pose origin = source.state.pose;
          origin.yaw = exit.yaw;
          branch_ = BranchEvent{origin, exit, intent, 1};
          clear_optimization();
        }
      }
    } catch (...) {
      // Normal input validation and failure diagnostics remain authoritative.
    }
  }
  bool completed = false;
  if (branch_) {
    auto & event = *branch_;
    if (
      event.phase == 1 && stopped &&
      std::abs(wrap_angle(source.state.pose.yaw - event.origin.yaw)) < .03) {
      event.phase = 2;
      clear_optimization();
    }
    if (event.phase == 2 && stopped) {
      const auto target = swerve_chassis_core::inverse(
        config_.geometry, {event.alignment.vx, event.alignment.vy, event.alignment.wz},
        source.state.steering);
      bool aligned = true;
      for (std::size_t w = 0; w < 4; ++w)
        aligned &= std::abs(target.steering[w] - source.state.steering[w]) <
                   .8 * config_.limits.tracking_tolerance;
      if (aligned) {
        event.phase = 3;
        clear_optimization();
      }
    }
    if (event.phase == 3 && distance(source.state.pose, event.exit) < .04) {
      event.origin = source.state.pose;
      event.origin.yaw = event.exit.yaw;
      event.phase = 4;
      clear_optimization();
    }
    if (
      event.phase == 4 && stopped &&
      std::abs(wrap_angle(source.state.pose.yaw - event.exit.yaw)) < .01) {
      branch_.reset();
      clear_optimization();
      completed = true;
    }
  }
  int phase = 0;
  if (completed) {
    // Hold this measured checkpoint for the handoff cycle. The caller rejoins
    // its ordered route before the following compute; no stale crop can pull
    // the robot back toward an earlier sampled tangent.
    phase = 4;
    input.path = {source.state.pose};
    input.goal = source.state.pose;
    input.goal_active = true;
  }
  if (branch_) {
    const auto & event = *branch_;
    phase = event.phase;
    input.path =
      phase == 3 ? std::vector<Pose>{event.origin, event.exit} : std::vector<Pose>{event.origin};
    input.goal = phase == 3 ? event.exit : event.origin;
    input.goal_active = true;
    if (phase == 2) input.alignment = event.alignment;
  }
  auto result = compute_tracking(input);
  result.branch_phase = phase;
  result.completed_transition = completed;
  if (phase)
    result.motion_reference =
      MotionReference{input.path, input.goal, input.goal_active, input.alignment};
  result.compute_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  return result;
}
Result Controller::compute_tracking(const Input & input)
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
    !finite(input.state.velocity) || !finite(input.goal) ||
    (input.alignment && !finite(*input.alignment)) || !std::isfinite(input.dt) || input.dt <= 0 ||
    input.dt > 1 ||
    std::any_of(input.path.begin(), input.path.end(), [](Pose p) { return !finite(p); }) ||
    !std::isfinite(PathView(input.path).length())) {
    result.diagnostics.invalid_input = 1;
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
  std::unique_ptr<ReferenceProfile> reference;
  std::vector<TrackingCost> breakdown;
  CriticContext context{input, config_, nullptr, &breakdown};
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
    reference = std::make_unique<ReferenceProfile>(input.path, config_);
    context.reference = reference.get();
    bool wheels_stopped = true;
    for (double v : input.state.wheel_speeds)
      wheels_stopped &= std::abs(v) < config_.limits.stopped_speed;
    if (reorientation_) {
      const auto target = swerve_chassis_core::inverse(
        config_.geometry, {reorientation_->vx, reorientation_->vy, reorientation_->wz},
        input.state.steering);
      bool aligned = true;
      for (std::size_t w = 0; w < 4; ++w)
        aligned &= std::abs(target.steering[w] - input.state.steering[w]) <
                   .8 * config_.limits.tracking_tolerance;
      if (aligned || !wheels_stopped) {
        reorientation_.reset();
        warm_.clear();
        warm_elapsed_ = 0;
      }
    }
    if (wheels_stopped && !reorientation_) {
      const auto match = PathView(input.path).start_match(input.state.pose);
      const double speed = reference->speed(match.arc);
      const bool nominal_start =
        match.error < .02 && std::abs(wrap_angle(match.pose.yaw - input.state.pose.yaw)) < .06;
      auto unit = reference->unit(match.arc, nominal_start ? match.pose.yaw : input.state.pose.yaw);
      const double branch = reference->next_branch(match.arc);
      if (branch >= 0 && branch < .10 && match.error < config_.path_corridor) {
        // Stop before a representation boundary, then select its outgoing
        // branch. Reusing the incoming tangent has no way to release the stop.
        const double departure = std::min(PathView(input.path).length(), match.arc + branch + .06);
        unit = reference->unit(departure, PathView(input.path).at(departure).yaw);
      }
      Velocity intent{speed * unit.vx, speed * unit.vy, speed * unit.wz};
      if (!nominal_start) {
        // Reorientation must preserve the same recovery direction as the
        // feedback seed. Latching only the tangent while off the path makes
        // alignment and cross-track correction request opposite branches.
        const double ex = config_.cross_gain * (match.pose.x - input.state.pose.x);
        const double ey = config_.cross_gain * (match.pose.y - input.state.pose.y);
        intent.vx += std::cos(input.state.pose.yaw) * ex + std::sin(input.state.pose.yaw) * ey;
        intent.vy += -std::sin(input.state.pose.yaw) * ex + std::cos(input.state.pose.yaw) * ey;
        intent.wz += 2.5 * wrap_angle(match.pose.yaw - input.state.pose.yaw);
        intent = model_.project(intent);
      }
      if (std::hypot(intent.vx, intent.vy) + std::abs(intent.wz) > .01) {
        const auto target = swerve_chassis_core::inverse(
          config_.geometry, {intent.vx, intent.vy, intent.wz}, input.state.steering);
        for (std::size_t w = 0; w < 4; ++w)
          if (
            std::abs(target.steering[w] - input.state.steering[w]) >
            config_.limits.tracking_tolerance)
            reorientation_ = intent;
      }
    }
    context.initial_intent =
      input.alignment ? &*input.alignment : (reorientation_ ? &*reorientation_ : nullptr);
    context.alignment_only = input.alignment.has_value();
    for (auto & critic : critics_) {
      try {
        critic->prepare(context);
      } catch (...) {
        ++result.diagnostics.critic_errors;
        throw;
      }
    }
    auto fresh = seed(input, *reference, reorientation_);
    if (input.alignment) {
      const auto target = swerve_chassis_core::inverse(
        config_.geometry, {input.alignment->vx, input.alignment->vy, input.alignment->wz},
        input.state.steering);
      double peak = 1e-9;
      for (double v : target.speed) peak = std::max(peak, std::abs(v));
      const double scale = .5 * config_.limits.stopped_speed / peak;
      fresh.assign(
        config_.horizon,
        {scale * input.alignment->vx, scale * input.alignment->vy, scale * input.alignment->wz});
    }
    const double reference_arc = PathView(input.path).start_match(input.state.pose).arc;
    result.reference_speed = reference->speed(reference_arc);
    result.reference_limit = reference->limiting_reason(reference_arc);
    result.branch_distance = reference->next_branch(reference_arc);
    const auto initial_intent = swerve_chassis_core::inverse(
      config_.geometry, {fresh.front().vx, fresh.front().vy, fresh.front().wz},
      input.state.steering);
    bool stopped = true;
    for (double v : input.state.wheel_speeds) stopped &= std::abs(v) < config_.limits.stopped_speed;
    for (std::size_t w = 0; w < 4; ++w)
      result.presteering |=
        stopped && std::abs(initial_intent.steering[w] - input.state.steering[w]) >
                     config_.limits.tracking_tolerance;
    result.reorientation_active = reorientation_.has_value();
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
    result.noise_scale = noise_scale * alignment;
    bool rotation_only = PathView(input.path).length() < 1e-6;
    const bool hold_rotation_position =
      rotation_only && std::abs(wrap_angle(input.path.back().yaw - input.state.pose.yaw)) > .02;
    auto mean =
      !input.alignment && !reorientation_ && warm_.size() == config_.horizon ? warm_ : fresh;
    auto score = [&](const std::vector<Trajectory> & traces) {
      std::vector<double> costs(traces.size(), 0);
      std::vector<Rejection> rejected(traces.size(), Rejection::None);
      for (std::size_t i = 0; i < traces.size(); ++i)
        if (!traces[i].valid) {
          rejected[i] = traces[i].rejection;
          costs[i] = std::numeric_limits<double>::infinity();
        }
      for (const auto & critic : critics_) {
        try {
          critic->score(context, traces, costs);
          if (costs.size() != traces.size()) throw std::runtime_error("critic changed batch size");
        } catch (...) {
          ++result.diagnostics.critic_errors;
          throw;
        }
        for (std::size_t i = 0; i < traces.size(); ++i)
          if (!std::isfinite(costs[i]) && rejected[i] == Rejection::None)
            rejected[i] = critic->rejection_reason();
      }
      for (std::size_t i = 0; i < traces.size(); ++i) {
        // A later custom critic must not revive a rejected candidate.
        if (!traces[i].valid || rejected[i] != Rejection::None)
          costs[i] = std::numeric_limits<double>::infinity();
        switch (rejected[i]) {
          case Rejection::InvalidState:
            ++result.diagnostics.invalid_state;
            break;
          case Rejection::InvalidControl:
            ++result.diagnostics.invalid_control;
            break;
          case Rejection::ActuatorResidual:
            ++result.diagnostics.actuator_residual;
            break;
          case Rejection::GeneratorError:
            ++result.diagnostics.generator_error;
            break;
          case Rejection::Collision:
            ++result.diagnostics.collision_rejected;
            break;
          case Rejection::Critic:
            ++result.diagnostics.critic_rejected;
            break;
          case Rejection::None:
            break;
        }
      }
      result.evaluated += traces.size();
      return costs;
    };
    for (std::size_t iteration = 0; iteration < config_.iterations; ++iteration) {
      std::vector<Trajectory> batch;
      batch.reserve(config_.samples);
      for (std::size_t k = 0; k < config_.samples; ++k) {
        auto candidate = k == 1 ? fresh : mean;
        // Always regenerate a slew-limited brake from this cycle's state. It
        // goes through the same rollout and collision checks as every sample.
        if (k == 2) candidate.assign(config_.horizon, {});
        // A short validated departure can make progress without committing
        // the entire horizon through another steering event. It is scored
        // alongside the full seed and brake, never forced onto the executor.
        if (k == 3)
          for (auto & u : candidate) {
            u.vx *= .7;
            u.vy *= .7;
            u.wz *= .7;
          }
        if (k == 4) {
          candidate = fresh;
          for (std::size_t i = std::min<std::size_t>(5, candidate.size()); i < candidate.size();
               ++i)
            candidate[i] = {};
        }
        Velocity noise;
        if (k >= 5 && !reorientation_ && !input.alignment)
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
        if (hold_rotation_position)
          for (auto & u : candidate) {
            u.vx = 0;
            u.vy = 0;
          }
        batch.push_back(model_.rollout(input.state, condition(std::move(candidate))));
      }
      const auto costs = score(batch);
      for (std::size_t k = 0; k < std::min<std::size_t>(5, batch.size()); ++k)
        result.candidates[k] = {
          costs[k], std::isfinite(costs[k]),
          breakdown.size() == batch.size() ? breakdown[k] : TrackingCost{}};
      const auto winner =
        static_cast<std::size_t>(std::min_element(costs.begin(), costs.end()) - costs.begin());
      if (!std::isfinite(costs[winner])) break;
      if (costs[winner] < best) {
        best = costs[winner];
        selected = batch[winner].controls;
        selected_trace = batch[winner];
        result.candidates[5] = {
          best, true, breakdown.size() == batch.size() ? breakdown[winner] : TrackingCost{}};
        result.braking_selected = winner == 2;
      }
      std::vector<Velocity> weighted(config_.horizon);
      double total = 0, squared = 0;
      for (std::size_t k = 0; k < batch.size(); ++k) {
        const double w = std::exp(-(costs[k] - costs[winner]) / config_.temperature);
        total += w;
        squared += w * w;
        if (w == 0) continue;
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
        result.candidates[5] = {cost, true, breakdown.size() == 1 ? breakdown[0] : TrackingCost{}};
        selected = weighted;
        selected_trace = std::move(update.front());
        result.braking_selected = winner == 2 && result.effective_samples < 1 + 1e-9;
      }
      mean = cost <= costs[winner] + .02 ? weighted : batch[winner].controls;
    }
  } catch (...) {
    if (!result.diagnostics.critic_errors) ++result.diagnostics.controller_errors;
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
  result.controls = selected;
  warm_ = std::move(selected);
  return finish();
}
// Publication intervals can be shorter than compute-start intervals when the
// preceding cycle took longer. Recondition the complete selected trajectory and
// re-run all hard checks before exposing an adjusted first request.
bool Controller::retime(const Input & input, double dt, Result & result)
{
  const auto begin = std::chrono::steady_clock::now();
  if (
    result.status != Status::Ok || result.controls.empty() || !std::isfinite(dt) || dt <= 0 ||
    dt > 1)
    return false;
  try {
    auto controls = result.controls;
    auto previous = input.state.requested;
    auto steering = input.state.generator.command.steering;
    for (std::size_t i = 0; i < controls.size(); ++i) {
      controls[i] = model_.condition(controls[i], previous, steering, i ? config_.model_dt : dt);
      steering = swerve_chassis_core::inverse(
                   config_.geometry, {controls[i].vx, controls[i].vy, controls[i].wz}, steering)
                   .steering;
      previous = controls[i];
    }
    auto trace = model_.rollout(input.state, controls);
    if (!trace.valid) return false;
    auto timed = input;
    timed.dt = dt;
    if (result.motion_reference) {
      timed.path = result.motion_reference->path;
      timed.goal = result.motion_reference->goal;
      timed.goal_active = result.motion_reference->goal_active;
      timed.alignment = result.motion_reference->alignment;
    }
    ReferenceProfile reference(timed.path, config_);
    CriticContext context{timed, config_, &reference};
    context.initial_intent =
      timed.alignment ? &*timed.alignment : (reorientation_ ? &*reorientation_ : nullptr);
    context.alignment_only = timed.alignment.has_value();
    std::vector<TrackingCost> breakdown;
    context.breakdown = &breakdown;
    std::vector<Trajectory> batch{trace};
    std::vector<double> costs{0};
    for (auto & critic : critics_) {
      critic->prepare(context);
      critic->score(context, batch, costs);
      if (costs.size() != 1 || !std::isfinite(costs[0])) return false;
    }
    result.command.velocity = controls.front();
    result.controls = controls;
    result.cost = costs.front();
    result.candidates[5] = {
      result.cost, true, breakdown.size() == 1 ? breakdown[0] : TrackingCost{}};
    result.prediction.clear();
    for (const auto & state : trace.states) result.prediction.push_back(state.pose);
    warm_ = std::move(controls);
    result.compute_ms +=
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    return true;
  } catch (...) {
    return false;
  }
}
}  // namespace swerve_mppi
