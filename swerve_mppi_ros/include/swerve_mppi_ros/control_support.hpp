#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include "swerve_mppi/heading_reference.hpp"
#include "swerve_mppi/ordered_path.hpp"
#include "swerve_mppi/types.hpp"

namespace swerve_mppi_ros
{
using namespace swerve_mppi;
using GoalTolerance = swerve_mppi::GoalTolerance;
// Task completion belongs to the caller. No completion flag exists in core.
class GoalChecker
{
public:
  explicit GoalChecker(GoalTolerance tolerance = {}) : t_(tolerance)
  {
    if (!valid_goal_tolerance(t_)) throw std::invalid_argument("invalid goal tolerance");
  }
  void reset() { since_.reset(); }
  bool reached(const State & state, const Pose & goal, double stamp)
  {
    bool inside = inside_goal(state, goal, t_);
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
