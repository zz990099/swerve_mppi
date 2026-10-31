#include "swerve_mppi/controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/heading_reference.hpp"
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
Velocity feedback_request(
  const Config & config, const State & state, const PathMatch & match, double speed, Velocity unit,
  const Input & input)
{
  const bool positioned = terminal_position_reached(input) && !input.precise_heading;
  const double ex = positioned ? 0 : config.cross_gain * (match.pose.x - state.pose.x);
  const double ey = positioned ? 0 : config.cross_gain * (match.pose.y - state.pose.y);
  if (positioned) speed = 0;
  return {
    speed * unit.vx + std::cos(state.pose.yaw) * ex + std::sin(state.pose.yaw) * ey,
    speed * unit.vy - std::sin(state.pose.yaw) * ex + std::cos(state.pose.yaw) * ey,
    free_heading(input)
      ? input.goal_heading_weight * 2.5 * heading_feedback(input.goal.yaw, state.pose.yaw)
      : speed * unit.wz + 2.5 * heading_feedback(match.pose.yaw, state.pose.yaw)};
}
bool stable_nominal(
  const Config & config, const State & state, const PathMatch & match, double speed,
  Velocity nominal, Velocity feedback)
{
  if (
    speed <= .01 || match.error >= .02 ||
    std::abs(wrap_angle(match.pose.yaw - state.pose.yaw)) >= .06)
    return false;
  const auto a = swerve_chassis_core::inverse(
    config.geometry, {nominal.vx, nominal.vy, nominal.wz}, state.generator.command.steering);
  const auto b = swerve_chassis_core::inverse(
    config.geometry, {feedback.vx, feedback.vy, feedback.wz}, state.generator.command.steering);
  for (std::size_t w = 0; w < 4; ++w)
    if (
      std::abs(std::remainder(a.steering[w] - b.steering[w], swerve_chassis_core::pi)) >
      .5 * config.limits.tracking_tolerance)
      return false;
  return true;
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
  tracking_start_ = false;
}
std::vector<Velocity> Controller::seed(
  const Input & input, const ReferenceProfile & reference, const std::optional<Velocity> & intent,
  SeedStyle style, SeedDiagnostic * diagnostic) const
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
    Velocity desired = feedback_request(config_, s, m, speed, unit, input);
    if (style == SeedStyle::Alignment && free_heading(input)) {
      const double direction = std::clamp(heading_feedback(m.pose.yaw, s.pose.yaw), -1., 1.);
      desired.wz += (1 - input.goal_heading_weight) * reference.heading_preference(progress) * .35 *
                    config_.max_yaw_rate * direction;
    }
    if (style == SeedStyle::Rotation) {
      desired.vx = 0;
      desired.vy = 0;
      // Predict a connected stop-then-turn trajectory. An immediate spin on a
      // moving rolling branch can be projected into unwanted translation.
      if (
        std::hypot(s.velocity.vx, s.velocity.vy) > input.goal_tolerance.linear_speed ||
        std::hypot(s.requested.vx, s.requested.vy) > 1e-4)
        desired = {};
    }
    if (style == SeedStyle::Position) {
      const double dx = config_.cross_gain * (input.goal.x - s.pose.x);
      const double dy = config_.cross_gain * (input.goal.y - s.pose.y);
      desired = {
        std::cos(s.pose.yaw) * dx + std::sin(s.pose.yaw) * dy,
        -std::sin(s.pose.yaw) * dx + std::cos(s.pose.yaw) * dy, 0};
      if (
        std::abs(s.velocity.wz) > input.goal_tolerance.angular_speed ||
        std::abs(s.requested.wz) > 1e-4)
        desired = {};
    }
    // Filter only corrections smaller than the wheel tracking tolerance.
    // A fixed 2 cm pose gate can hide a large steering correction at low speed.
    bool stopped = true;
    for (double v : s.wheel_speeds) stopped &= std::abs(v) < config_.limits.stopped_speed;
    const auto nominal_unit =
      reference.unit(progress, free_heading(input) ? s.pose.yaw : m.pose.yaw);
    Velocity nominal{speed * nominal_unit.vx, speed * nominal_unit.vy, speed * nominal_unit.wz};
    if (!free_heading(input) && stopped && stable_nominal(config_, s, m, speed, nominal, desired))
      desired = nominal;
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
    if (input.departure)
      desired = {
        speed * input.departure->vx, speed * input.departure->vy, speed * input.departure->wz};
    auto v = model_.condition(
      desired, s.requested, s.generator.command.steering, i ? config_.model_dt : input.dt);
    if (diagnostic) {
      if (!i) {
        diagnostic->desired_first = desired;
        diagnostic->conditioned_first = v;
      }
      diagnostic->desired_peak = std::max(
        diagnostic->desired_peak, std::hypot(desired.vx, desired.vy) + .3 * std::abs(desired.wz));
      diagnostic->conditioned_peak =
        std::max(diagnostic->conditioned_peak, std::hypot(v.vx, v.vy) + .3 * std::abs(v.wz));
    }
    out.push_back(v);
    auto next = model_.rollout(s, {v});
    if (next.valid) s = next.states.back();
  }
  return out;
}
Result Controller::compute(const Input & external)
{
  const auto begin = std::chrono::steady_clock::now();
  Input source = external;
  try {
    if (
      source.reference_kind == ReferenceKind::ExternalPath && !source.path.empty() &&
      source.path.size() <= 4096) {
      source.path = geometric_path(source.path, config_.lookahead);
      source.heading_preferences.clear();
      source.reference_kind = PathView(source.path).length() < 1e-8 ? ReferenceKind::FixedHeading
                                                                    : ReferenceKind::GeometricPath;
    }
    if (!source.path.empty()) source.goal_heading_weight = terminal_heading_weight(source, config_);
  } catch (const std::invalid_argument &) {
    source.path.clear();
  }
  // Validate the caller's reference before a retained maneuver can replace it.
  // In particular, a canceled/empty route must not execute stale alignment.
  if (
    source.path.empty() || source.path.size() > 4096 || !valid_heading_preferences(source) ||
    !model_.valid(source.state) || !finite(source.state.velocity) || !finite(source.goal) ||
    !std::isfinite(source.remaining_arc) || source.remaining_arc < -1 ||
    (source.alignment && !finite(*source.alignment)) ||
    (source.departure && !finite(*source.departure)) || !std::isfinite(source.dt) ||
    source.dt <= 0 || source.dt > 1 ||
    std::any_of(source.path.begin(), source.path.end(), [](Pose p) { return !finite(p); }) ||
    !std::isfinite(PathView(source.path).length()))
    return compute_tracking(source);
  Input input = source;
  if (std::hypot(source.state.velocity.vx, source.state.velocity.vy) > .04) tracking_start_ = false;
  bool stopped = true;
  for (double v : source.state.wheel_speeds) stopped &= std::abs(v) < config_.limits.stopped_speed;
  // Branch events are short, ordered maneuvers in the same continuous twist
  // space. Their location and exit are held until measured completion.
  if (
    !free_heading(source) && !branch_ && !tracking_start_ && stopped && !source.path.empty() &&
    source.path.size() <= 4096 && model_.valid(source.state) &&
    std::all_of(source.path.begin(), source.path.end(), [](Pose p) { return finite(p); })) {
    try {
      PathView path(source.path);
      ReferenceProfile reference(source, config_);
      const auto match = path.start_match(source.state.pose);
      const auto desired_unit = reference.unit(match.arc, match.pose.yaw);
      const auto actual_unit = reference.unit(match.arc, source.state.pose.yaw);
      const double speed = reference.speed(match.arc);
      const Velocity nominal{
        speed * desired_unit.vx, speed * desired_unit.vy, speed * desired_unit.wz};
      auto feedback = feedback_request(config_, source.state, match, speed, actual_unit, source);
      if (stable_nominal(config_, source.state, match, speed, nominal, feedback))
        feedback = nominal;
      const auto nominal_wheels = swerve_chassis_core::inverse(
        config_.geometry, {nominal.vx, nominal.vy, nominal.wz}, source.state.steering);
      const auto feedback_wheels = swerve_chassis_core::inverse(
        config_.geometry, {feedback.vx, feedback.vy, feedback.wz}, source.state.steering);
      bool current_branch = false;
      for (std::size_t w = 0; w < 4; ++w)
        current_branch |= std::abs(nominal_wheels.steering[w] - source.state.steering[w]) >
                            swerve_chassis_core::pi / 2 ||
                          std::abs(nominal_wheels.steering[w] - feedback_wheels.steering[w]) >
                            swerve_chassis_core::pi / 2;
      double boundary = reference.next_branch(match.arc);
      if (current_branch && (boundary < 0 || boundary > .10)) boundary = 0;
      if (
        boundary >= 0 && boundary < .10 && match.error < 2 * config_.path_corridor && speed > .01) {
        const double departure =
          std::min(path.length(), match.arc + boundary + std::max(.08, boundary + .04));
        Pose exit = path.at(departure);
        const double yaw_change = wrap_angle(exit.yaw - match.pose.yaw);
        const double dx = exit.x - source.state.pose.x, dy = exit.y - source.state.pose.y;
        const double length = std::hypot(dx, dy);
        Velocity intent{
          std::cos(exit.yaw) * dx + std::sin(exit.yaw) * dy,
          -std::sin(exit.yaw) * dx + std::cos(exit.yaw) * dy, 0};
        if (length > 1e-4) {
          intent.vx /= length;
          intent.vy /= length;
          intent.wz = yaw_change / length;
        }
        bool corridor = true;
        for (int i = 1; i <= 10; ++i) {
          const double u = i / 10.;
          Pose point{source.state.pose.x + u * dx, source.state.pose.y + u * dy, exit.yaw};
          corridor &=
            path.match(point, match.arc, departure).error < std::max(.025, match.error + .005);
        }
        if (
          corridor && length > .02 && path.next_corner(match.arc) >= departure &&
          std::abs(wrap_angle(exit.yaw - source.state.pose.yaw)) < config_.heading_corridor) {
          Pose origin = source.state.pose;
          const bool align_body =
            std::abs(wrap_angle(match.pose.yaw - source.state.pose.yaw)) > .05;
          if (align_body) origin.yaw = match.pose.yaw;
          branch_ = BranchEvent{origin, exit, intent, align_body ? 1 : 2};
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
      std::abs(wrap_angle(source.state.pose.yaw - event.origin.yaw)) < .003) {
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
        event.origin = source.state.pose;
        event.exit.yaw =
          wrap_angle(event.origin.yaw + event.alignment.wz * distance(event.origin, event.exit));
        event.phase = 3;
        clear_optimization();
      }
    }
    const double length = distance(event.origin, event.exit);
    const double dx = event.exit.x - event.origin.x, dy = event.exit.y - event.origin.y;
    const double px = source.state.pose.x - event.origin.x,
                 py = source.state.pose.y - event.origin.y;
    const double along = length > 1e-6 ? (px * dx + py * dy) / length : length;
    const double cross = length > 1e-6 ? std::abs(px * dy - py * dx) / length : 0;
    if (event.phase == 3 && along >= length - .04 && cross < .06) {
      event.origin = source.state.pose;
      event.origin.yaw = PathView(source.path).start_match(source.state.pose).pose.yaw;
      event.settle_heading = std::abs(wrap_angle(event.origin.yaw - source.state.pose.yaw)) > .03;
      event.phase = 4;
      clear_optimization();
    }
    if (
      event.phase == 4 &&
      (!event.settle_heading ||
       (stopped && std::abs(wrap_angle(source.state.pose.yaw - event.origin.yaw)) < .003))) {
      // A heading correction leaves the wheels in a spin configuration.
      // Let ordinary feedback-consistent presteering resume translation before
      // interpreting that wheel pose as another spatial branch event.
      tracking_start_ = event.settle_heading;
      branch_.reset();
      clear_optimization();
      completed = true;
    }
  }
  int phase = 0;
  if (branch_) {
    const auto & event = *branch_;
    phase = event.phase;
    input.path =
      phase == 3 ? std::vector<Pose>{event.origin, event.exit} : std::vector<Pose>{event.origin};
    input.goal = phase == 3 ? event.exit : event.origin;
    input.goal_active = true;
    if (phase == 2) input.alignment = event.alignment;
    if (phase == 3) {
      input.departure = event.alignment;
      input.goal_active = false;
    }
    input.heading_preferences.clear();
    input.precise_heading = phase == 1 || phase == 4;
    input.reference_kind =
      phase == 2 ? ReferenceKind::WheelAlignment
                 : (phase == 3 ? ReferenceKind::Departure : ReferenceKind::FixedHeading);
    input.goal_heading_weight = phase == 1 || phase == 4 ? 1. : 0.;
  }
  auto result = compute_tracking(input);
  result.branch_phase = phase;
  result.completed_transition = completed;
  result.goal_heading_weight = input.goal_heading_weight;
  result.motion_reference = MotionReference{input.reference_kind,
                                            input.goal_heading_weight,
                                            input.remaining_arc,
                                            input.path,
                                            input.goal,
                                            input.goal_active,
                                            input.alignment,
                                            input.precise_heading,
                                            input.departure,
                                            input.goal_tolerance,
                                            input.heading_preferences};
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
    input.path.empty() || input.path.size() > 4096 || !valid_heading_preferences(input) ||
    !valid_goal_tolerance(input.goal_tolerance) || !model_.valid(input.state) ||
    !finite(input.state.velocity) || !finite(input.goal) || !std::isfinite(input.remaining_arc) ||
    input.remaining_arc < -1 || (input.alignment && !finite(*input.alignment)) ||
    (input.departure && !finite(*input.departure)) || !std::isfinite(input.dt) || input.dt <= 0 ||
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
  const bool terminal_hold =
    terminal && terminal_position_reached(input) && !input.precise_heading && !input.alignment &&
    inside_goal(input.state, input.goal, input.goal_tolerance) &&
    std::all_of(input.state.wheel_speeds.begin(), input.state.wheel_speeds.end(), [](double v) {
      return std::abs(v) <= .05;
    });
  const double noise_scale =
    terminal ? std::clamp(
                 std::max(
                   distance(input.state.pose, input.goal) / config_.approach_distance,
                   std::abs(wrap_angle(input.state.pose.yaw - input.goal.yaw)) / 0.5),
                 0.02, 1.0)
             : 1.0;
  double best = std::numeric_limits<double>::infinity();
  std::vector<Velocity> selected;
  std::size_t selected_decision = 0;
  std::vector<Rejection> last_rejections;
  Trajectory selected_trace;
  try {
    reference = std::make_unique<ReferenceProfile>(input, config_);
    context.reference = reference.get();
    bool wheels_stopped = true;
    for (double v : input.state.wheel_speeds)
      wheels_stopped &= std::abs(v) < config_.limits.stopped_speed;
    if (input.departure || terminal_hold || terminal_position_reached(input))
      reorientation_.reset();
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
    if (
      !free_heading(input) && wheels_stopped && !reorientation_ && !input.departure &&
      !terminal_hold) {
      const auto match = PathView(input.path).start_match(input.state.pose);
      const double speed = reference->speed(match.arc);
      const auto nominal_unit = reference->unit(match.arc, match.pose.yaw);
      const auto actual_unit = reference->unit(match.arc, input.state.pose.yaw);
      const Velocity nominal{
        speed * nominal_unit.vx, speed * nominal_unit.vy, speed * nominal_unit.wz};
      const auto feedback =
        feedback_request(config_, input.state, match, speed, actual_unit, input);
      const bool nominal_start =
        stable_nominal(config_, input.state, match, speed, nominal, feedback);
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
        intent = feedback_request(config_, input.state, match, speed, unit, input);
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
      input.alignment
        ? &*input.alignment
        : (input.departure ? &*input.departure : (reorientation_ ? &*reorientation_ : nullptr));
    context.alignment_only = input.alignment.has_value();
    for (auto & critic : critics_) {
      try {
        critic->prepare(context);
      } catch (...) {
        ++result.diagnostics.critic_errors;
        throw;
      }
    }
    result.terminal_hold = terminal_hold;
    result.alignment_intent = reorientation_;
    if (reorientation_) {
      const auto wheels = swerve_chassis_core::inverse(
        config_.geometry, {reorientation_->vx, reorientation_->vy, reorientation_->wz},
        input.state.steering);
      for (std::size_t w = 0; w < 4; ++w)
        result.alignment_error =
          std::max(result.alignment_error, std::abs(wheels.steering[w] - input.state.steering[w]));
    }
    auto fresh = seed(
      input, *reference, reorientation_, SeedStyle::Tracking,
      input.record_decisions ? &result.seeds[0] : nullptr);
    auto guided = free_heading(input) && !reorientation_
                    ? seed(
                        input, *reference, {}, SeedStyle::Alignment,
                        input.record_decisions ? &result.seeds[1] : nullptr)
                    : fresh;
    auto unlocked = free_heading(input) && reorientation_ ? seed(input, *reference) : fresh;
    const bool terminal_pose = free_heading(input) && terminal &&
                               ordered_remaining(input) <= config_.goal_heading_full_distance;
    const bool terminal_rotation = free_heading(input) && terminal && input.goal_heading_weight > 0;
    auto rotate = terminal_rotation ? seed(input, *reference, {}, SeedStyle::Rotation) : fresh;
    auto position = terminal_pose ? seed(input, *reference, {}, SeedStyle::Position) : fresh;
    if (terminal_hold) {
      fresh.assign(config_.horizon, {});
      guided = fresh;
      unlocked = fresh;
      rotate = fresh;
      position = fresh;
    }
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
    result.heading_preference = free_heading(input) ? (1 - input.goal_heading_weight) *
                                                        reference->heading_preference(reference_arc)
                                                    : 0;
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
      !input.alignment && (rotation_only || terminal_position_reached(input)) &&
      std::abs(wrap_angle(input.goal.yaw - input.state.pose.yaw)) > input.goal_tolerance.yaw;
    auto mean = !terminal_hold && !input.alignment && !input.departure && !reorientation_ &&
                    warm_.size() == config_.horizon
                  ? warm_
                  : fresh;
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
      if (input.record_decisions) last_rejections = rejected;
      result.evaluated += traces.size();
      return costs;
    };
    auto decision = [&](
                      const Trajectory & trajectory, const std::vector<Velocity> & raw,
                      std::size_t iteration, std::size_t candidate, double cost,
                      TrackingCost tracking, Rejection reason) {
      DecisionDiagnostic d;
      d.iteration = iteration;
      d.candidate = candidate;
      d.valid = std::isfinite(cost);
      d.rejection = reason;
      d.cost = cost;
      d.tracking = tracking;
      if (!raw.empty()) d.raw_first = raw.front();
      if (!trajectory.controls.empty()) d.conditioned_first = trajectory.controls.front();
      for (const auto & u : raw)
        d.raw_peak = std::max(d.raw_peak, std::hypot(u.vx, u.vy) + .3 * std::abs(u.wz));
      for (const auto & u : trajectory.controls)
        d.conditioned_peak =
          std::max(d.conditioned_peak, std::hypot(u.vx, u.vy) + .3 * std::abs(u.wz));
      PathView path(input.path);
      const double start = path.start_match(input.state.pose).arc;
      double progress = start;
      for (const auto & state : trajectory.states)
        progress =
          path
            .match_ordered(
              state.pose, progress,
              std::min(path.length(), progress + config_.max_speed * config_.model_dt * 1.5 + .02))
            .arc;
      d.predicted_progress = progress - start;
      if (!trajectory.states.empty()) {
        d.xy_motion = distance(input.state.pose, trajectory.states.back().pose);
        d.yaw_motion =
          std::abs(wrap_angle(input.state.pose.yaw - trajectory.states.back().pose.yaw));
      }
      result.decisions.push_back(d);
    };
    for (std::size_t iteration = 0; iteration < config_.iterations; ++iteration) {
      std::vector<Trajectory> batch;
      batch.reserve(config_.samples);
      std::vector<std::vector<Velocity>> raw_candidates;
      if (input.record_decisions) raw_candidates.reserve(config_.samples);
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
        // Keep-body and optional gentle-alignment proposals compete under the
        // same costs and physical checks. Neither is imposed on every sample.
        if (free_heading(input) && k >= 5 && k % 2) candidate = guided;
        if (free_heading(input) && k == 6) candidate = unlocked;
        if (terminal_rotation && k == 7) candidate = rotate;
        if (terminal_pose && k == 8) candidate = position;
        Velocity noise;
        if (k >= 9 && !terminal_hold && !reorientation_ && !input.alignment && !input.departure)
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
        if (input.record_decisions) raw_candidates.push_back(candidate);
        batch.push_back(model_.rollout(input.state, condition(std::move(candidate))));
      }
      const auto costs = score(batch);
      const auto offset = result.decisions.size();
      if (input.record_decisions)
        for (std::size_t k = 0; k < batch.size(); ++k)
          decision(
            batch[k], raw_candidates[k], iteration, k, costs[k],
            breakdown.size() == batch.size() ? breakdown[k] : TrackingCost{}, last_rejections[k]);
      for (std::size_t k = 0; k < std::min<std::size_t>(5, batch.size()); ++k)
        result.candidates[k] = {
          costs[k], std::isfinite(costs[k]),
          breakdown.size() == batch.size() ? breakdown[k] : TrackingCost{}};
      const auto winner =
        static_cast<std::size_t>(std::min_element(costs.begin(), costs.end()) - costs.begin());
      if (!std::isfinite(costs[winner])) break;
      if (costs[winner] < best) {
        if (input.record_decisions) selected_decision = offset + winner;
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
      auto raw_weighted = input.record_decisions ? weighted : std::vector<Velocity>{};
      weighted = condition(std::move(weighted));
      std::vector<Trajectory> update{model_.rollout(input.state, weighted)};
      const double cost = score(update).front();
      if (input.record_decisions)
        decision(
          update.front(), raw_weighted, iteration, config_.samples, cost,
          breakdown.size() == 1 ? breakdown[0] : TrackingCost{}, last_rejections.front());
      if (cost <= best + .02) {
        if (input.record_decisions) selected_decision = result.decisions.size() - 1;
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
  // Only the optimizer's accepted request may latch an ordinary wheel-alignment
  // intent. This is execution continuity, not a prescribed body-heading target.
  if (free_heading(input) && !terminal_hold) {
    bool stopped = true;
    for (double speed : input.state.wheel_speeds)
      stopped &= std::abs(speed) < config_.limits.stopped_speed;
    if (stopped) {
      const auto & u = selected.front();
      const auto target =
        swerve_chassis_core::inverse(config_.geometry, {u.vx, u.vy, u.wz}, input.state.steering);
      bool turning = false, driving = false;
      for (std::size_t w = 0; w < 4; ++w) {
        turning |= std::abs(target.steering[w] - input.state.steering[w]) >
                   config_.limits.tracking_tolerance;
        driving |= std::abs(target.speed[w]) > 2 * config_.limits.stopped_speed;
      }
      if (turning && driving) {
        // Wheel direction is homogeneous in twist. Preserve the optimizer's
        // accepted direction, not a tiny slew-limited launch amplitude.
        double amplitude = 0;
        for (const auto & v : selected)
          amplitude = std::max(amplitude, std::hypot(v.vx, v.vy) + .3 * std::abs(v.wz));
        const double scale =
          std::max(1., amplitude / std::max(1e-9, std::hypot(u.vx, u.vy) + .3 * std::abs(u.wz)));
        reorientation_ = model_.project({scale * u.vx, scale * u.vy, scale * u.wz});
      }
    }
  }
  if (input.record_decisions && selected_decision < result.decisions.size())
    result.decisions[selected_decision].selected = true;
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
      timed.reference_kind = result.motion_reference->reference_kind;
      timed.goal_heading_weight = result.motion_reference->goal_heading_weight;
      timed.goal_tolerance = result.motion_reference->goal_tolerance;
      timed.heading_preferences = result.motion_reference->heading_preferences;
      timed.remaining_arc = result.motion_reference->remaining_arc;
      timed.path = result.motion_reference->path;
      timed.goal = result.motion_reference->goal;
      timed.goal_active = result.motion_reference->goal_active;
      timed.alignment = result.motion_reference->alignment;
      timed.precise_heading = result.motion_reference->precise_heading;
      timed.departure = result.motion_reference->departure;
    }
    ReferenceProfile reference(timed, config_);
    CriticContext context{timed, config_, &reference};
    context.initial_intent =
      timed.alignment
        ? &*timed.alignment
        : (timed.departure ? &*timed.departure : (reorientation_ ? &*reorientation_ : nullptr));
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
