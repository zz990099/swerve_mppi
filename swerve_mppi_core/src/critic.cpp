#include "swerve_mppi/critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "swerve_mppi/model.hpp"

namespace swerve_mppi
{
namespace
{
struct Match
{
  double error = 0, progress = 0, heading = 0;
};
Match match(const Pose & p, const std::vector<Pose> & path)
{
  Match result{distance(p, path.front()), 0, path.front().yaw};
  double length = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1];
    const auto & b = path[i];
    double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    double t = l2 > 1e-12 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / l2, 0.0, 1.0) : 0;
    double error = std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
    if (error < result.error)
      result = {
        error, length + t * std::sqrt(l2), wrap_angle(a.yaw + t * wrap_angle(b.yaw - a.yaw))};
    length += std::sqrt(l2);
  }
  return result;
}
class TrackingCritic final : public Critic
{
public:
  explicit TrackingCritic(TrackingWeights weights) : w_(weights)
  {
    for (double x :
         {w_.path_distance, w_.path_heading, w_.progress, w_.goal_position, w_.goal_heading,
          w_.effort, w_.smoothness, w_.steering_change})
      if (!std::isfinite(x) || x < 0) throw std::invalid_argument("invalid critic weight");
  }
  void score(
    const CriticContext & c, const std::vector<Trajectory> & batch,
    std::vector<double> & costs) const override
  {
    const bool terminal = distance(c.input.path.back(), c.input.goal) < 0.001;
    const double start_progress = match(c.input.state.pose, c.input.path).progress;
    for (std::size_t k = 0; k < batch.size(); ++k) {
      const auto & t = batch[k];
      if (!t.valid || t.states.empty()) continue;
      Velocity previous = c.input.state.velocity;
      if (t.mode == c.input.state.mode && !t.controls.empty()) {
        const auto wheels = Model(c.config).inverse(t.controls.front(), c.input.state.steering);
        for (std::size_t wheel = 0; wheel < 4; ++wheel)
          costs[k] +=
            w_.steering_change * std::pow(wheels.angles[wheel] - c.input.state.steering[wheel], 2);
      }
      for (std::size_t i = 1; i < t.states.size(); ++i) {
        const auto & s = t.states[i];
        auto m = match(s.pose, c.input.path);
        double heading = wrap_angle(s.pose.yaw - m.heading);
        double near =
          terminal
            ? std::clamp(1 - distance(s.pose, c.input.goal) / c.config.approach_distance, 0.0, 1.0)
            : 0;
        double gyaw = wrap_angle(s.pose.yaw - c.input.goal.yaw);
        const auto & u = t.controls[i - 1];
        double smooth = std::pow(u.vx - previous.vx, 2) + std::pow(u.vy - previous.vy, 2) +
                        std::pow(u.wz - previous.wz, 2);
        costs[k] +=
          c.config.model_dt *
          (w_.path_distance * m.error * m.error + w_.path_heading * (1 - near) * heading * heading +
           near * w_.goal_position * std::pow(distance(s.pose, c.input.goal), 2) +
           near * w_.goal_heading * gyaw * gyaw +
           w_.effort * (u.vx * u.vx + u.vy * u.vy + u.wz * u.wz) + w_.smoothness * smooth);
        previous = u;
      }
      const auto & last = t.states.back();
      costs[k] += w_.progress * (start_progress - match(last.pose, c.input.path).progress);
      if (terminal)
        costs[k] += w_.goal_position * std::pow(distance(last.pose, c.input.goal), 2) +
                    w_.goal_heading * std::pow(wrap_angle(last.pose.yaw - c.input.goal.yaw), 2);
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
