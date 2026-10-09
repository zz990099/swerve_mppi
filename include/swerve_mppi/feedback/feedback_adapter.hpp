#pragma once
#include <string>
#include <vector>

#include "swerve_mppi/feedback/feedback.hpp"

namespace swerve_mppi
{
struct StampedPose
{
  Pose2d pose;  // Fixed world/odometry frame, never the body frame.
  TimestampNs stamp_ns = kInvalidTimestamp;
};
// ROS-independent representation of ONE complete JointState observation.
// Position is rad; velocity is rad/s. All required joints share stamp_ns.
// At most 64 names are admitted; required joints must occur exactly once.
struct JointObservation
{
  TimestampNs stamp_ns = kInvalidTimestamp;
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
};
enum class SnapshotError
{
  None,
  InvalidTime,
  Unsynchronized,
  Stale,
  Future,
  MissingJoint,
  InvalidFeedback
};
struct SnapshotResult
{
  SnapshotError error = SnapshotError::InvalidFeedback;
  std::optional<VehicleState> state;
};
// Maps named joints to FL,FR,RL,RR, converts wheel rad/s to linear m/s, and
// derives body twist from the SAME encoder sample. Never reads odometry twist,
// fills missing joints from history, predicts a pose or rewrites a source
// stamp.
class FeedbackAdapter
{
public:
  explicit FeedbackAdapter(const Config & config, std::string joint_prefix = "");
  SnapshotResult make(
    const JointObservation & joints, const StampedPose & pose, const ModeFeedback & mode,
    TimestampNs planning_stamp_ns) const;

private:
  SnapshotResult assemble(
    const JointObservation & joints, const Pose2d & pose, const ModeFeedback & mode,
    TimestampNs stamp_ns) const;
  Config config_;
  std::array<std::string, 8> names_;
};
}  // namespace swerve_mppi
