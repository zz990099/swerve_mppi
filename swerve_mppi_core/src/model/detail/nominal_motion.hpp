#pragma once
#include <cmath>

#include "swerve_mppi/model/model.hpp"
namespace swerve_mppi::detail
{
inline void integrate_constant(Pose2d & pose, const Twist2d & v, double dt)
{
  const double a = v.wz * dt;
  const double sinc = std::abs(a) < 1e-4 ? 1 - a * a / 6 + a * a * a * a / 120 : std::sin(a) / a;
  const double cosc =
    std::abs(a) < 1e-4 ? a / 2 - a * a * a / 24 + a * a * a * a * a / 720 : (1 - std::cos(a)) / a;
  const double dx = dt * (sinc * v.vx - cosc * v.vy);
  const double dy = dt * (cosc * v.vx + sinc * v.vy);
  const double yaw = wrap_angle(pose.yaw);
  pose.x += std::cos(yaw) * dx - std::sin(yaw) * dy;
  pose.y += std::sin(yaw) * dx + std::cos(yaw) * dy;
  pose.yaw = wrap_angle(yaw + a);
}
inline void append_motion(
  const StepResult & step, std::vector<Pose2d> * poses, std::vector<double> * margins,
  double & position_error)
{
  if (poses) {
    poses->push_back(step.state.pose);
  }
  if (margins) {
    margins->push_back(position_error + step.sweep_margin_m);
  }
  position_error += step.integration_error_m;
}
}  // namespace swerve_mppi::detail
