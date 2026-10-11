#pragma once
#include <cmath>

#include "swerve_mppi/model/model.hpp"
namespace swerve_mppi::detail
{
inline bool within_body_limits(const Twist2d & v, DriveMode mode, const Config & c)
{
  constexpr double tolerance = 1e-9;
  switch (mode) {
    case DriveMode::DualAckermann:
      return std::abs(v.vx) <= c.max_vx_mps + tolerance &&
             std::abs(v.wz) <= c.max_yaw_rate_radps + tolerance;
    case DriveMode::Crab:
      return std::hypot(v.vx, v.vy) <= c.max_crab_speed_mps + tolerance;
    case DriveMode::Spin:
      return std::abs(v.wz) <= c.max_spin_radps + tolerance;
  }
  return false;
}
}  // namespace swerve_mppi::detail
