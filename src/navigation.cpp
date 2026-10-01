#include "swerve_mppi/navigation.hpp"
#include "validation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_mppi {
namespace {
bool same_path(const std::vector<Pose2d> &a, const std::vector<Pose2d> &b) {
  if (a.size() != b.size())
    return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (a[i].x != b[i].x || a[i].y != b[i].y || a[i].yaw != b[i].yaw)
      return false;
  return true;
}
double distance_to_motion(const Pose2d &point, const Pose2d &from, const Pose2d &to) {
  const double dx = to.x - from.x, dy = to.y - from.y;
  const double length2 = dx * dx + dy * dy;
  const double t =
      length2 > 1e-12
          ? std::clamp(((point.x - from.x) * dx + (point.y - from.y) * dy) / length2, 0.0, 1.0)
          : 0.0;
  return std::hypot(point.x - from.x - t * dx, point.y - from.y - t * dy);
}
} // namespace
PathManager::PathManager(const Config &config) : config_(config) { validate(config); }
Pose2d PathManager::interpolate(double distance) const {
  if (distance >= lengths_.back())
    return path_.back();
  const auto next = std::upper_bound(lengths_.begin(), lengths_.end(), distance);
  const auto i = static_cast<std::size_t>(next - lengths_.begin());
  if (i == 0)
    return path_.front();
  const double t = (distance - lengths_[i - 1]) / (lengths_[i] - lengths_[i - 1]);
  return {path_[i - 1].x + t * (path_[i].x - path_[i - 1].x),
          path_[i - 1].y + t * (path_[i].y - path_[i - 1].y),
          wrap_angle(path_[i - 1].yaw + t * angle_distance(path_[i].yaw, path_[i - 1].yaw))};
}
PathReference PathManager::update(const ControllerInput &input) {
  if (!detail::valid_input(input, config_))
    throw std::invalid_argument("invalid path input");
  PathReference out;
  out.changed = path_id_ != input.path_id || heading_policy_ != input.heading_policy ||
                !same_path(path_, input.reference_path);
  if (out.changed) {
    path_ = input.reference_path;
    lengths_.assign(path_.size(), 0);
    for (std::size_t i = 1; i < path_.size(); ++i) {
      lengths_[i] =
          lengths_[i - 1] + std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
      if (!std::isfinite(lengths_[i])) {
        reset();
        throw std::invalid_argument("path length overflow");
      }
    }
    path_id_ = input.path_id;
    heading_policy_ = input.heading_policy;
    progress_ = 0;
    segment_ = 1;
    while (segment_ < path_.size() && lengths_[segment_] < 1e-12)
      ++segment_;
    previous_pose_ = input.vehicle.pose;
  }
  const auto &pose = input.vehicle.pose;
  const double displacement = std::hypot(pose.x - previous_pose_.x, pose.y - previous_pose_.y);
  const double window = out.changed ? config_.path_search_window_m
                                    : std::min(config_.path_search_window_m,
                                               displacement + config_.path_progress_slack_m);
  const double upper = std::min(lengths_.back(), progress_ + window);
  // Match the current segment first. A later segment becomes eligible only
  // after measured motion captures/passes their shared endpoint, in waypoint order.
  // Initial matching stays on the first nonzero segment even for short loops.
  for (std::size_t i = segment_; i < path_.size() && lengths_[i - 1] <= upper; ++i) {
    const double length = lengths_[i] - lengths_[i - 1];
    if (length < 1e-12) {
      segment_ = i + 1;
      continue;
    }
    const double low = std::max(0.0, (progress_ - lengths_[i - 1]) / length);
    const double high = std::min(1.0, (upper - lengths_[i - 1]) / length);
    const double dx = path_[i].x - path_[i - 1].x, dy = path_[i].y - path_[i - 1].y;
    const double projection =
        ((pose.x - path_[i - 1].x) * dx + (pose.y - path_[i - 1].y) * dy) / (length * length);
    const double t = std::clamp(projection, low, high);
    progress_ = std::max(progress_, lengths_[i - 1] + t * length);
    std::size_t next = i + 1;
    while (next < path_.size() && lengths_[next] - lengths_[i] < 1e-12)
      ++next;
    const double bx = next < path_.size() ? path_[next].x - path_[i].x : dx;
    const double by = next < path_.size() ? path_[next].y - path_[i].y : dy;
    const bool sharp = std::abs(std::atan2(dx * by - dy * bx, dx * bx + dy * by)) >
                       config_.path_lookahead_turn_rad;
    // Smooth sampling points are passed by crossing their endpoint plane; they
    // are not mandatory precision waypoints. Sharp corners still need XY capture.
    const bool passed = !sharp && projection >= 1.0;
    const bool captured = !out.changed && lengths_[i] <= upper + 1e-9 &&
                          lengths_[i] - progress_ <= config_.goal_position_tolerance_m &&
                          (passed || distance_to_motion(path_[i], previous_pose_, pose) <=
                                         config_.goal_position_tolerance_m);
    if (!captured)
      break;
    progress_ = lengths_[i];
    segment_ = i + 1;
  }
  double end = std::min(lengths_.back(), progress_ + config_.path_lookahead_m);
  // Do not look through a reversal or sharp corner: its distant endpoint can
  // point backwards before the corner has actually been reached.
  double accumulated_turn = 0;
  for (std::size_t i = segment_; i + 1 < path_.size() && lengths_[i] < end; ++i) {
    if (lengths_[i] < progress_ - 1e-9 || lengths_[i] >= end)
      continue;
    const double ax = path_[i].x - path_[i - 1].x, ay = path_[i].y - path_[i - 1].y;
    std::size_t next = i + 1;
    while (next < path_.size() && lengths_[next] - lengths_[i] < 1e-12)
      ++next;
    if (next == path_.size())
      continue;
    const double bx = path_[next].x - path_[i].x, by = path_[next].y - path_[i].y;
    if (std::hypot(ax, ay) < 1e-12 || std::hypot(bx, by) < 1e-12)
      continue;
    accumulated_turn += std::abs(std::atan2(ax * by - ay * bx, ax * bx + ay * by));
    if (accumulated_turn > config_.path_lookahead_turn_rad) {
      if (lengths_[i] - progress_ <= config_.goal_position_tolerance_m &&
          std::hypot(pose.x - path_[i].x, pose.y - path_[i].y) <=
              config_.goal_position_tolerance_m) {
        progress_ = lengths_[i];
        end = std::min(lengths_.back(), progress_ + config_.path_lookahead_m);
        accumulated_turn = 0;
      } else {
        end = lengths_[i];
        out.corner_target = true;
        break;
      }
    }
  }
  const auto nearest = interpolate(progress_);
  out.cross_track_error_m = std::hypot(pose.x - nearest.x, pose.y - nearest.y);
  out.local_path.push_back(nearest);
  const auto local_begin =
      std::upper_bound(lengths_.begin(), lengths_.end(), progress_ + 1e-9) - lengths_.begin();
  for (std::size_t i = static_cast<std::size_t>(local_begin);
       i < path_.size() && lengths_[i] < end - 1e-9; ++i)
    out.local_path.push_back(path_[i]);
  if (end > progress_ + 1e-9)
    out.local_path.push_back(interpolate(end));
  out.goal = path_.back();
  out.progress_m = progress_;
  out.remaining_m = lengths_.back() - progress_;
  out.terminal = end >= lengths_.back() - 1e-9;
  previous_pose_ = pose;
  return out;
}
void PathManager::reset() {
  path_.clear();
  lengths_.clear();
  progress_ = 0;
  segment_ = 1;
}

GoalManager::GoalManager(const Config &config) : config_(config) { validate(config); }
GoalState GoalManager::update(const VehicleState &s, const PathReference &path, bool busy) {
  if (!detail::valid_vehicle(s, config_) || !std::isfinite(path.goal.x) ||
      !std::isfinite(path.goal.y) || !std::isfinite(path.goal.yaw) ||
      !std::isfinite(path.remaining_m) || path.remaining_m < 0 || !std::isfinite(path.progress_m) ||
      path.progress_m < 0)
    throw std::invalid_argument("invalid goal input");
  GoalState out;
  out.distance_m = std::hypot(s.pose.x - path.goal.x, s.pose.y - path.goal.y);
  out.yaw_error_rad = angle_distance(path.goal.yaw, s.pose.yaw);
  const double yaw = std::abs(out.yaw_error_rad);
  if (!position_acquired_ && out.distance_m <= config_.goal_position_tolerance_m &&
      path.remaining_m <= config_.goal_position_tolerance_m)
    position_acquired_ = true;
  if (out.distance_m > 2 * config_.goal_position_tolerance_m ||
      (out.distance_m > config_.goal_position_tolerance_m &&
       yaw <= config_.goal_yaw_tolerance_rad && is_stopped(s, config_)))
    position_acquired_ = false;
  out.position_acquired = position_acquired_;
  if (!busy && position_acquired_ && out.distance_m <= config_.goal_position_tolerance_m &&
      yaw <= config_.goal_yaw_tolerance_rad && s.mode_confirmed && !s.mode_fault &&
      is_stopped(s, config_)) {
    if (settle_start_ < 0)
      settle_start_ = s.stamp_s;
    if (s.stamp_s - settle_start_ + 1e-9 >= config_.goal_settle_time_s)
      complete_ = true;
  } else
    settle_start_ = -1;
  out.complete = complete_;
  out.status =
      complete_            ? NavigationStatus::Complete
      : position_acquired_ ? (yaw > config_.goal_yaw_tolerance_rad ? NavigationStatus::AligningGoal
                                                                   : NavigationStatus::Settling)
      : path.remaining_m < config_.goal_slowdown_distance_m ? NavigationStatus::ApproachingGoal
                                                            : NavigationStatus::Tracking;
  if (progress_stamp_ < 0 || busy || complete_ ||
      (out.status == NavigationStatus::Settling && is_stopped(s, config_)) ||
      path.progress_m - last_progress_ >= config_.progress_distance_m ||
      last_distance_ - out.distance_m >= config_.progress_distance_m ||
      (position_acquired_ && last_yaw_error_ - yaw >= config_.goal_yaw_tolerance_rad)) {
    progress_stamp_ = s.stamp_s;
    last_progress_ = path.progress_m;
    last_distance_ = out.distance_m;
    last_yaw_error_ = yaw;
  }
  out.stalled = !complete_ && s.stamp_s - progress_stamp_ >= config_.progress_timeout_s;
  return out;
}
void GoalManager::reset() {
  position_acquired_ = complete_ = false;
  settle_start_ = progress_stamp_ = -1;
  last_progress_ = last_distance_ = last_yaw_error_ = 0;
}
} // namespace swerve_mppi
