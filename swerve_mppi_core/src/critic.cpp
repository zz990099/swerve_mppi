#include "swerve_mppi/critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/path.hpp"
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
          w.smoothness, w.steering_change, w.consistency})
      if (!std::isfinite(x) || x < 0) throw std::invalid_argument("invalid critic weight");
  }
  void score(
    const CriticContext & c, const std::vector<Trajectory> & batch,
    std::vector<double> & costs) const override
  {
    PathView path(c.input.path);
    auto start = path.match(c.input.state.pose);
    bool terminal = c.input.goal_active && distance(c.input.path.back(), c.input.goal) < .001;
    double initial_heading = std::abs(wrap_angle(c.input.state.pose.yaw - start.pose.yaw));
    for (std::size_t k = 0; k < batch.size(); ++k) {
      const auto & t = batch[k];
      if (!t.valid || t.states.empty()) continue;
      double duration = c.config.model_dt * t.controls.size(), progress = start.arc;
      double norm = 1. / std::max<std::size_t>(1, t.controls.size());
      costs[k] += (w_.steering_change * t.actuator_cost + w_.consistency * t.residual_cost) /
                  std::max(.1, duration);
      auto previous = c.input.state.requested;
      PathMatch m = start;
      for (std::size_t i = 1; i < t.states.size(); ++i) {
        const auto & s = t.states[i];
        m = path.match(
          s.pose, progress,
          std::min(path.length(), progress + c.config.max_speed * c.config.model_dt * 1.5 + .02));
        progress = m.arc;
        double heading = wrap_angle(s.pose.yaw - m.pose.yaw);
        if (
          m.error > std::max(c.config.path_corridor, start.error + .005) ||
          std::abs(heading) > std::max(c.config.heading_corridor, initial_heading + .005)) {
          costs[k] = std::numeric_limits<double>::infinity();
          break;
        }
        const auto & u = t.controls[i - 1];
        double dt = i == 1 ? c.input.dt : c.config.model_dt;
        double smooth =
          std::pow(
            std::hypot(u.vx - previous.vx, u.vy - previous.vy) /
              (c.config.limits.linear_acceleration * dt),
            2) +
          std::pow((u.wz - previous.wz) / (c.config.limits.angular_acceleration * dt), 2);
        costs[k] +=
          norm * (w_.path_distance * std::pow(m.error / c.config.position_scale, 2) +
                  w_.path_heading * std::pow(heading / c.config.heading_scale, 2) +
                  w_.effort * (u.vx * u.vx + u.vy * u.vy + u.wz * u.wz) + w_.smoothness * smooth);
        previous = u;
      }
      double reference_distance =
        std::max(.1, std::min(path.length() - start.arc, c.config.max_speed * duration));
      costs[k] -= w_.progress * (progress - start.arc) / reference_distance;
      if (terminal) {
        // Remaining ordered distance replaces direct attraction through a curve/corner.
        double remaining = path.length() - progress;
        costs[k] += w_.goal_position * std::pow(remaining / c.config.approach_distance, 2);
        if (remaining < .02)
          costs[k] +=
            w_.goal_heading *
            std::pow(
              wrap_angle(t.states.back().pose.yaw - c.input.goal.yaw) / c.config.heading_scale, 2);
      }
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
