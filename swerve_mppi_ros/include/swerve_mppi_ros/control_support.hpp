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
struct GoalTolerance : GoalPoseTolerance
{
  double linear_speed = 0.035, angular_speed = 0.035, settle = 0.3;
};
// Task completion belongs to the caller. No completion flag exists in core.
class GoalChecker
{
public:
  explicit GoalChecker(GoalTolerance tolerance = {}) : t_(tolerance)
  {
    if (!t_.valid()) throw std::invalid_argument("invalid goal pose tolerance");
    for (double v : {t_.linear_speed, t_.angular_speed, t_.settle})
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid goal tolerance");
  }
  void reset() { since_.reset(); }
  bool reached(const State & state, const Pose & goal, double stamp)
  {
    bool inside = t_.contains(state.pose, goal) &&
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
  std::optional<double> accepted_command_stamp;
};

inline std::optional<LongitudinalRequestSample> longitudinal_request_sample(
  const std::optional<LongitudinalRequestSample> & previous, std::optional<double> speed, double dt)
{
  if (!speed) return std::nullopt;
  if (previous && std::isfinite(dt) && dt >= 1e-6 && dt <= 1)
    return LongitudinalRequestSample{*speed, (*speed - previous->speed) / dt, dt};
  return LongitudinalRequestSample{*speed, std::nullopt, 0};
}

inline double command_interval(
  double now, double last_return, Velocity last_request, const ChassisFeedback & feedback)
{
  // Nav2 stamps cmd_vel after the plugin returns. Use the stamp echoed by the
  // chassis when it matches that request. Otherwise allow a clock tick for the handoff.
  const auto & accepted = feedback.state.requested;
  const bool confirmed = feedback.accepted_command_stamp &&
                         *feedback.accepted_command_stamp >= last_return - 1e-9 &&
                         accepted.vx == last_request.vx && accepted.vy == last_request.vy &&
                         accepted.wz == last_request.wz;
  const double previous = confirmed ? *feedback.accepted_command_stamp : last_return + .005;
  return std::max(1e-6, now - previous);
}
}  // namespace swerve_mppi_ros
