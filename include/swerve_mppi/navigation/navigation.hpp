#pragma once
#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi
{
enum class PathTargetKind
{
  Lookahead,
  Corner,
  Goal
};
struct PathReference
{
  std::vector<Pose2d> local_path;
  Pose2d goal;
  double progress_m = 0;
  double remaining_m = 0;
  double cross_track_error_m = 0;
  bool changed = false;
  bool terminal = false;
  bool corner_target = false;
  // Effective translation target; an uncaptured corner takes precedence over
  // the global goal even when that goal is nearby in XY.
  Pose2d target;
  PathTargetKind target_kind = PathTargetKind::Lookahead;
  double target_remaining_m = 0;
  bool goal_eligible = false;
};

// Stateful arc-length matching in a bounded forward window. No global nearest
// point search at crossings and no implicit reversal of waypoint order.
class PathManager
{
public:
  explicit PathManager(const Config & config);
  PathReference update(const ControllerInput & input);
  void reset();

private:
  Pose2d interpolate(double distance) const;
  Config config_;
  std::vector<Pose2d> path_;
  std::vector<double> lengths_;
  std::uint64_t path_id_ = 0;
  PathHeadingPolicy heading_policy_ = PathHeadingPolicy::FollowPath;
  Pose2d previous_pose_;
  double progress_ = 0;
  // Endpoint of the current segment. Arc projection alone cannot advance it.
  std::size_t segment_ = 1;
};

struct GoalState
{
  NavigationStatus status = NavigationStatus::Tracking;
  double distance_m = 0;
  double yaw_error_rad = 0;
  bool position_acquired = false;
  bool complete = false;
  bool stalled = false;
};
class GoalManager
{
public:
  explicit GoalManager(const Config & config);
  GoalState update(const VehicleState & state, const PathReference & path, bool execution_busy);
  void reset();

private:
  Config config_;
  bool position_acquired_ = false;
  bool complete_ = false;
  double settle_start_ = -1;
  double last_observation_s_ = -1;
  double progress_stamp_ = -1;
  double last_progress_ = 0;
  double last_distance_ = 0;
  double last_yaw_error_ = 0;
};
}  // namespace swerve_mppi
