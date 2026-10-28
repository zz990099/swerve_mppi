#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include "swerve_mppi/path.hpp"
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
    progress_arc_ = 0;
  }
  bool empty() const { return path_.empty(); }
  bool goal_active(double terminal_distance = .5) const
  {
    return !empty() && remaining() <= terminal_distance;
  }
  double remaining() const { return empty() ? 0 : PathView(path_).length() - progress_arc_; }
  Pose goal() const { return path_.back(); }
  std::vector<Pose> local(const Pose & pose, double length = 3.0)
  {
    PathView view(path_);
    // Only adjacent segments may advance ordered progress; crossings cannot skip a leg.
    while (progress_ + 1 < path_.size()) {
      const auto & a = path_[progress_];
      const auto & b = path_[progress_ + 1];
      double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
      double dot = (pose.x - a.x) * dx + (pose.y - a.y) * dy;
      double cross = (pose.x - a.x) * dy - (pose.y - a.y) * dx;
      bool passed = l2 < 1e-12 || (dot >= l2 && cross * cross <= .03 * .03 * l2);
      bool corner = view.turn(progress_ + 1) >= 1.0471975512 && distance(pose, b) < .015;
      if (!passed && !corner) break;
      progress_arc_ = view.vertex_arc(progress_ + 1);
      ++progress_;
    }
    double before = 0;
    for (std::size_t i = 1; i <= progress_; ++i) before += distance(path_[i - 1], path_[i]);
    double next = progress_ + 1 < path_.size()
                    ? before + distance(path_[progress_], path_[progress_ + 1])
                    : before;
    auto projected = view.match(pose, before, next);
    progress_arc_ = std::max(progress_arc_, projected.arc);
    double end = std::min(view.length(), progress_arc_ + length);
    std::vector<Pose> out{view.at(progress_arc_)};
    double arc = 0;
    for (std::size_t i = 1; i < path_.size(); ++i) {
      arc += distance(path_[i - 1], path_[i]);
      if (arc > progress_arc_ + 1e-8 && arc < end - 1e-8) out.push_back(path_[i]);
    }
    if (end > progress_arc_ + 1e-8) out.push_back(view.at(end));
    return out;
  }

private:
  std::vector<Pose> path_;
  std::size_t progress_ = 0;
  double progress_arc_ = 0;
};
struct ChassisFeedback
{
  State state;
  bool fault = false, feedback_valid = false;
  double feedback_stamp = 0;
};
}  // namespace swerve_mppi_ros
