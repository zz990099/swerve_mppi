#include "swerve_mppi/critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/path.hpp"
#include "swerve_mppi/reference.hpp"
namespace swerve_mppi
{
namespace
{
class TrackingCritic final : public Critic
{
public:
  explicit TrackingCritic(TrackingWeights w) : w_(w)
  {
    for (double x :
         {w.path_distance, w.progress, w.goal_position, w.goal_heading, w.alignment_heading,
          w.effort, w.smoothness, w.steering_change, w.consistency, w.reference_velocity,
          w.presteering_alignment, w.longitudinal_jerk})
      if (!std::isfinite(x) || x < 0) throw std::invalid_argument("invalid critic weight");
  }
  void score(
    const CriticContext & c, const std::vector<Trajectory> & batch,
    std::vector<double> & costs) const override
  {
    if (c.breakdown) c.breakdown->assign(batch.size(), {});
    PathView path(c.input.path);
    const auto start = path.start_match(c.input.state.pose);
    std::unique_ptr<ReferenceProfile> local_reference;
    if (!c.reference)
      local_reference = std::make_unique<ReferenceProfile>(
        c.input.path, c.config, c.input.speed_preview, start.arc, c.input.dt);
    const auto & reference = c.reference ? *c.reference : *local_reference;
    const double heading_scale =
      c.input.precise_heading ? std::min(.005, c.config.heading_scale) : c.config.heading_scale;
    const double checkpoint = path.next_corner(start.arc);
    const bool preview_reaches_goal = c.input.reaches_goal();
    const bool terminal = c.input.goal_active && preview_reaches_goal;
    const bool terminal_heading_active =
      terminal && path.length() - start.arc <= c.config.goal_heading_activation_distance;
    const auto initial_heading = reference.heading(start.arc, preview_reaches_goal);
    const auto initial_match =
      match_geometric_heading(c.input.state.pose.yaw, initial_heading.yaw, c.config);
    const bool use_jerk =
      w_.longitudinal_jerk > 0 && !terminal && !c.input.departure && path.length() > 1e-6 &&
      (!c.input.speed_preview.plan ||
       !c.input.speed_preview.plan->near_corner(c.input.speed_preview.start_arc + start.arc)) &&
      (checkpoint == path.length() || checkpoint - start.arc > c.config.approach_distance);
    const double jerk_scale = c.config.limits.linear_acceleration / c.config.model_dt;
    const bool allow_turn_first = !c.input.departure && prefers_heading_alignment(
                                                          initial_heading.strength, initial_match,
                                                          checkpoint - start.arc, c.config);
    const double velocity_weight = allow_turn_first ? .05 : 1.;
    for (std::size_t k = 0; k < batch.size(); ++k) {
      const auto & t = batch[k];
      if (!t.valid || t.states.empty()) continue;
      TrackingCost detail;
      if (c.alignment_only && c.initial_intent) {
        const auto target = swerve_chassis_core::inverse(
          c.config.geometry, {c.initial_intent->vx, c.initial_intent->vy, c.initial_intent->wz},
          c.input.state.steering);
        const double norm = 1. / std::max<std::size_t>(1, t.controls.size());
        for (std::size_t i = 1; i < t.states.size(); ++i) {
          const auto & state = t.states[i];
          detail.path +=
            norm * w_.path_distance *
            std::pow(distance(state.pose, c.input.state.pose) / c.config.position_scale, 2);
          detail.heading +=
            norm * w_.alignment_heading *
            std::pow(
              wrap_angle(state.pose.yaw - c.input.state.pose.yaw) / c.config.heading_scale, 2);
          for (std::size_t w = 0; w < 4; ++w)
            detail.alignment += norm * w_.presteering_alignment *
                                std::pow((target.steering[w] - state.steering[w]) / .5, 2) / 4;
        }
        detail.effort = w_.steering_change * t.actuator_cost + w_.consistency * t.residual_cost;
        costs[k] += detail.path + detail.heading + detail.alignment + detail.effort;
        if (c.breakdown) (*c.breakdown)[k] = detail;
        continue;
      }
      double duration = c.config.model_dt * t.controls.size(), progress = start.arc;
      double norm = 1. / std::max<std::size_t>(1, t.controls.size());
      detail.effort += (w_.steering_change * t.actuator_cost + w_.consistency * t.residual_cost) /
                       std::max(.1, duration);
      auto previous = c.input.state.requested;
      auto previous_acceleration = c.input.previous_request_acceleration;
      auto previous_world = rotate_velocity(
        previous,
        previous_acceleration ? previous_acceleration->request_yaw : c.input.state.pose.yaw);
      PathMatch m = start;
      bool initial_alignment_done = false;
      for (double v : c.input.state.wheel_speeds)
        initial_alignment_done |= std::abs(v) >= c.config.limits.stopped_speed;
      for (std::size_t i = 1; i < t.states.size(); ++i) {
        const auto & s = t.states[i];
        m = path.match_ordered(
          s.pose, progress,
          std::min(path.length(), progress + c.config.max_speed * c.config.model_dt * 1.5 + .02));
        progress = m.arc;
        // Tracking corridors are soft objectives. Rejecting every temporarily
        // deviating prediction also rejects physically necessary recovery/braking.
        // Collision and actuator feasibility remain independent hard checks.
        const double slack =
          std::pow(std::max(0., m.error - c.config.path_corridor) / c.config.position_scale, 2);
        const auto & u = t.controls[i - 1];
        const auto unit = reference.unit(progress, s.pose.yaw);
        const double along = s.velocity.vx * unit.vx + s.velocity.vy * unit.vy;
        const auto heading = reference.heading(progress, preview_reaches_goal);
        const auto match = match_geometric_heading(s.pose.yaw, heading.yaw, c.config);
        double target_speed = reference.speed(progress);
        double alignment_error = 0;
        bool stopped = true;
        for (double v : s.wheel_speeds) stopped &= std::abs(v) < c.config.limits.stopped_speed;
        if (!stopped) initial_alignment_done = true;
        if (stopped && target_speed > .01) {
          const auto request = swerve_chassis_core::inverse(
            c.config.geometry, {u.vx, u.vy, u.wz}, s.generator.command.steering);
          bool reorienting = false;
          for (double v : request.speed)
            reorienting |= std::abs(v) > 2 * c.config.limits.stopped_speed;
          const auto alignment_intent =
            !initial_alignment_done && c.initial_intent ? *c.initial_intent : unit;
          const auto intent = swerve_chassis_core::inverse(
            c.config.geometry, {alignment_intent.vx, alignment_intent.vy, alignment_intent.wz},
            s.generator.command.steering);
          for (std::size_t w = 0; w < 4; ++w) {
            const double error = intent.steering[w] - s.steering[w];
            // Charge the measured initial reorientation until driving starts.
            // A later stop in this rollout belongs to a future branch event;
            // charging that angle at every remaining step rewards parking
            // before the event. Its physical motion still incurs actuator cost.
            if (!initial_alignment_done) alignment_error += std::pow(std::sin(error) / .5, 2) / 4;
            if (reorienting && std::abs(error) > c.config.limits.tracking_tolerance)
              target_speed = 0;  // Reference time pauses during necessary reorientation.
          }
        }
        const double excess_speed = std::max(0., along - target_speed);
        const double velocity_error = (along - target_speed) / .2;
        double dt = i == 1 ? c.input.dt : c.config.model_dt;
        if (use_jerk) {
          // Compare both accelerations in one tangent frame. Changing body yaw
          // or following a curve at constant speed should not look like a speed pulse.
          const double yaw = t.states[i - 1].pose.yaw;
          const auto world = rotate_velocity(u, yaw);
          const RequestAcceleration acceleration{
            (world.vx - previous_world.vx) / dt, (world.vy - previous_world.vy) / dt, dt, yaw};
          if (previous_acceleration) {
            const auto tangent = reference.unit(progress, 0);
            const double delta = tangent.vx * (acceleration.x - previous_acceleration->x) +
                                 tangent.vy * (acceleration.y - previous_acceleration->y);
            const double normalized = delta / (.5 * (dt + previous_acceleration->dt) * jerk_scale);
            // Give the command that will be published more weight than distant
            // rollout steps, which will be re-optimized next cycle.
            const double weight = i == 1 ? .25 + norm : norm;
            detail.longitudinal_jerk += w_.longitudinal_jerk * weight * normalized * normalized;
          }
          previous_acceleration = acceleration;
          previous_world = world;
        }
        double smooth =
          std::pow(
            std::hypot(u.vx - previous.vx, u.vy - previous.vy) /
              (c.config.limits.linear_acceleration * dt),
            2) +
          std::pow((u.wz - previous.wz) / (c.config.limits.angular_acceleration * dt), 2);
        detail.path +=
          norm * (4 * slack + w_.path_distance * std::pow(m.error / c.config.position_scale, 2));
        // A finite body-heading preference using the ordered path tangent.
        if (!c.input.departure && heading.strength > 0) {
          // When forward is preferred, backing along the path remains feasible
          // but should not win a straight segment solely on immediate progress.
          const double reverse_progress =
            allow_turn_first && !c.config.geometric_heading_allow_reverse && along > 0
              ? 6 * std::max(0., -s.velocity.vx) / std::max(.2, c.config.max_speed)
              : 0.;
          detail.heading += norm * c.config.geometric_heading_weight * heading.strength *
                            (match.loss + reverse_progress);
        }
        // During final rotation, returning at the horizon endpoint does not
        // compensate for an excursion earlier in the rollout. Keep endpoint
        // position a running soft objective throughout the terminal region.
        if (terminal_heading_active)
          detail.goal += norm * w_.goal_position *
                         std::pow(distance(s.pose, c.input.goal) / c.config.position_scale, 2);
        detail.velocity +=
          norm * (std::pow(excess_speed / .1, 2) +
                  w_.reference_velocity * velocity_weight * velocity_error * velocity_error);
        detail.alignment += norm * w_.presteering_alignment * alignment_error;
        detail.effort +=
          norm * (w_.effort * (u.vx * u.vx + u.vy * u.vy + u.wz * u.wz) + w_.smoothness * smooth);
        previous = u;
      }
      double reference_distance =
        std::max(.1, std::min(path.length() - start.arc, c.config.max_speed * duration));
      detail.progress -= w_.progress * (progress - start.arc) / reference_distance;
      if (
        checkpoint < path.length() - 1e-5 && checkpoint - start.arc < c.config.approach_distance) {
        // Reaching a mandatory corner is a task objective, even when the full
        // goal is still far away. Otherwise a short horizon can prefer parking
        // just before the corner to avoid paying the future steering cost.
        detail.goal += w_.goal_position *
                       std::pow(std::max(0., checkpoint - progress) / c.config.position_scale, 2);
      }
      if (terminal) {
        // Remaining ordered distance replaces direct attraction through a curve/corner.
        double remaining = path.length() - progress;
        detail.goal += w_.goal_position * std::pow(remaining / c.config.approach_distance, 2);
        // Intermediate waypoint yaw never contributes. The final yaw enters
        // only in the measured terminal region so translation remains orientation-free.
        const double heading_gate = terminal_heading_active ? 1. : 0.;
        if (terminal_heading_active)
          detail.goal +=
            w_.goal_position *
            std::pow(distance(t.states.back().pose, c.input.goal) / c.config.position_scale, 2);
        // A large final turn must not overwhelm position by its squared yaw
        // error. Huber loss retains quadratic precision near the target and
        // bounds the marginal incentive to rotate while far from its heading.
        const double yaw_error =
          std::abs(wrap_angle(t.states.back().pose.yaw - c.input.goal.yaw)) / heading_scale;
        detail.goal += heading_gate * w_.goal_heading *
                       (yaw_error <= 1. ? yaw_error * yaw_error : 2. * yaw_error - 1.);
      }
      costs[k] += detail.path + detail.heading + detail.velocity + detail.alignment +
                  detail.effort + detail.progress + detail.goal + detail.longitudinal_jerk;
      if (c.breakdown) (*c.breakdown)[k] = detail;
    }
  }

private:
  TrackingWeights w_;
};
}  // namespace
Critics tracking_critics(TrackingWeights weights)
{
  return {std::make_shared<TrackingCritic>(weights)};
}
}  // namespace swerve_mppi
