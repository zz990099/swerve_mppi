#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

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
                  std::abs(state.velocity.wz) <= t_.angular_speed && !state.switching;
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
class PathHandler
{
public:
  void set(std::vector<Pose> path)
  {
    if (path.empty() || path.size() > 4096) throw std::invalid_argument("empty or oversized path");
    for (auto p : path)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw))
        throw std::invalid_argument("invalid path");
    path_ = std::move(path);
    progress_ = 0;
  }
  bool empty() const { return path_.empty(); }
  bool goal_active() const { return !empty() && progress_ + 2 >= path_.size(); }
  Pose goal() const { return path_.back(); }
  std::vector<Pose> local(const Pose & pose, double length = 3.0)
  {
    // Search forward through nearby segments. Progress never jumps to a distant
    // crossing; dense paths can advance across multiple adjacent segments.
    while (progress_ + 1 < path_.size()) {
      const auto & a = path_[progress_];
      const auto & b = path_[progress_ + 1];
      double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
      bool passed = l2 < 1e-12 || ((pose.x - a.x) * dx + (pose.y - a.y) * dy >= l2);
      if (!passed && distance(pose, b) > 0.12) break;
      ++progress_;
    }
    std::vector<Pose> out{path_[progress_]};
    double travelled = 0;
    double forward_x = 0, forward_y = 0;
    for (std::size_t i = progress_ + 1; i < path_.size(); ++i) {
      const double dx = path_[i].x - path_[i - 1].x, dy = path_[i].y - path_[i - 1].y;
      if (dx * dx + dy * dy > 1e-12) {
        if (forward_x == 0 && forward_y == 0) {
          forward_x = dx;
          forward_y = dy;
        } else if (forward_x * dx + forward_y * dy <= 0) {
          // A monotonic local section prevents nearest-point scoring from
          // rewarding a later return leg before the intervening path is visited.
          break;
        }
      }
      travelled += distance(path_[i - 1], path_[i]);
      out.push_back(path_[i]);
      if (travelled >= length) break;
    }
    return out;
  }

private:
  std::vector<Pose> path_;
  std::size_t progress_ = 0;
};
struct ChassisFeedback
{
  State state;
  std::uint64_t request_id = 0;
  Velocity entry;
  bool fault = false;
};
struct WireCommand
{
  Command command;
  std::uint64_t request_id = 0;
  Velocity entry;
};
class ChassisAdapter
{
public:
  void observe(ChassisFeedback feedback)
  {
    feedback_ = feedback;
    // An accepted request that later faults needs a newer zero request.
    if (pending_ && feedback.fault && feedback.request_id >= pending_->request_id) pending_.reset();
    if (
      pending_ && feedback.request_id == pending_->request_id && !feedback.fault &&
      !feedback.state.switching && feedback.state.mode == pending_->command.mode)
      pending_.reset();
  }
  State state() const
  {
    auto state = feedback_.state;
    if (pending_) {
      state.switching = true;
      state.requested_mode = pending_->command.mode;
    }
    return state;
  }
  WireCommand translate(Command desired)
  {
    if (pending_) return *pending_;
    if (feedback_.fault || desired.mode != feedback_.state.mode) {
      if (feedback_.request_id == std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("chassis request ID exhausted");
      pending_ = WireCommand{
        {{}, feedback_.fault ? feedback_.state.mode : desired.mode},
        feedback_.request_id + 1,
        feedback_.fault ? Velocity{} : desired.velocity};
      return *pending_;
    }
    return {desired, feedback_.request_id, feedback_.entry};
  }

private:
  ChassisFeedback feedback_;
  std::optional<WireCommand> pending_;
};
}  // namespace swerve_mppi_ros
