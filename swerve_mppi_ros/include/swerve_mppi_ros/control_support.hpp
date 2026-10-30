#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include "swerve_mppi/ordered_path.hpp"
#include "swerve_mppi/types.hpp"

namespace swerve_mppi_ros
{
using namespace swerve_mppi;
struct GoalTolerance
{
  double position = 0.06, yaw = 0.05, linear_speed = 0.035, angular_speed = 0.035, settle = 0.3;
};
// Task completion belongs to the caller. No completion flag exists in core.
class GoalChecker
{
public:
  explicit GoalChecker(GoalTolerance tolerance = {}) : t_(tolerance)
  {
    for (double v : {t_.position, t_.yaw, t_.linear_speed, t_.angular_speed, t_.settle})
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid goal tolerance");
  }
  void reset() { since_.reset(); }
  bool reached(const State & state, const Pose & goal, double stamp)
  {
    bool inside = distance(state.pose, goal) <= t_.position &&
                  std::abs(wrap_angle(state.pose.yaw - goal.yaw)) <= t_.yaw &&
                  std::hypot(state.velocity.vx, state.velocity.vy) <= t_.linear_speed &&
                  std::abs(state.velocity.wz) <= t_.angular_speed;
    if (!inside || !std::isfinite(stamp)) {
      reset();
      return false;
    }
    if (!since_ || stamp < *since_) since_ = stamp;
    return stamp - *since_ >= t_.settle;
  }

private:
  GoalTolerance t_;
  std::optional<double> since_;
};
using PathHandler = swerve_mppi::OrderedPath;
struct ChassisFeedback
{
  State state;
  bool fault = false, feedback_valid = false;
  double feedback_stamp = 0;
};
}  // namespace swerve_mppi_ros
