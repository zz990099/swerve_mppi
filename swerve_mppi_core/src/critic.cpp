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
         {w.path_distance, w.path_heading, w.progress, w.goal_position, w.goal_heading, w.effort,
          w.smoothness, w.steering_change, w.consistency, w.reference_velocity,
          w.presteering_alignment})
      if (!std::isfinite(x) || x < 0) throw std::invalid_argument("invalid critic weight");
  }
  void score(
    const CriticContext & c, const std::vector<Trajectory> & batch,
    std::vector<double> & costs) const override
  {
    if (c.breakdown) c.breakdown->assign(batch.size(), {});
    PathView path(c.input.path);
    std::unique_ptr<ReferenceProfile> local_reference;
    if (!c.reference) local_reference = std::make_unique<ReferenceProfile>(c.input.path, c.config);
    const auto & reference = c.reference ? *c.reference : *local_reference;
    const double heading_scale =
      c.input.precise_heading ? std::min(.005, c.config.heading_scale) : c.config.heading_scale;
    auto start = path.start_match(c.input.state.pose);
    const double checkpoint = path.next_corner(start.arc);
    bool terminal = c.input.goal_active && distance(c.input.path.back(), c.input.goal) < .001;
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
            norm * w_.path_heading *
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
        double heading = wrap_angle(s.pose.yaw - m.pose.yaw);
        // Tracking corridors are soft objectives. Rejecting every temporarily
        // deviating prediction also rejects physically necessary recovery/braking.
        // Collision and actuator feasibility remain independent hard checks.
        const double slack =
          std::pow(std::max(0., m.error - c.config.path_corridor) / c.config.position_scale, 2) +
          std::pow(std::max(0., std::abs(heading) - c.config.heading_corridor) / heading_scale, 2);
        const auto & u = t.controls[i - 1];
        const auto unit = reference.unit(progress, s.pose.yaw);
        const double along = s.velocity.vx * unit.vx + s.velocity.vy * unit.vy;
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
        double smooth =
          std::pow(
            std::hypot(u.vx - previous.vx, u.vy - previous.vy) /
              (c.config.limits.linear_acceleration * dt),
            2) +
          std::pow((u.wz - previous.wz) / (c.config.limits.angular_acceleration * dt), 2);
        detail.path +=
          norm * (4 * slack + w_.path_distance * std::pow(m.error / c.config.position_scale, 2));
        detail.heading += norm * w_.path_heading * std::pow(heading / heading_scale, 2);
        detail.velocity += norm * (std::pow(excess_speed / .1, 2) +
                                   w_.reference_velocity * velocity_error * velocity_error);
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
        if (remaining < .02)
          detail.goal +=
            w_.goal_heading *
            std::pow(wrap_angle(t.states.back().pose.yaw - c.input.goal.yaw) / heading_scale, 2);
      }
      costs[k] += detail.path + detail.heading + detail.velocity + detail.alignment +
                  detail.effort + detail.progress + detail.goal;
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
