#pragma once
#include "swerve_mppi/feedback.hpp"
#include <string>
#include <vector>

namespace swerve_mppi {
struct StampedPose {
  Pose2d pose; // Fixed world/odometry frame, never the body frame.
  double stamp_s = -1;
  std::int64_t stamp_ns = -1; // Original transport stamp; used by make_at_nanoseconds.
};
// ROS-independent representation of ONE complete JointState observation.
// Position is rad; velocity is rad/s. All required joints must share stamp_s.
// At most 64 names are admitted; required joints must occur exactly once.
struct JointObservation {
  double stamp_s = -1;
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::int64_t stamp_ns = -1;
};
enum class SnapshotError { None, InvalidTime, Unsynchronized, MissingJoint, InvalidFeedback };
struct SnapshotResult {
  SnapshotError error = SnapshotError::InvalidFeedback;
  std::optional<VehicleState> state;
};
// Maps named joints to FL,FR,RL,RR, converts wheel rad/s to linear m/s, and
// derives body twist from the SAME encoder sample. Never reads odometry twist,
// fills missing joints from history, predicts a pose or rewrites a source stamp.
class FeedbackAdapter {
public:
  explicit FeedbackAdapter(const Config &config, std::string joint_prefix = "");
  SnapshotResult make(const JointObservation &joints, const StampedPose &pose,
                      const ModeFeedback &mode, double application_s) const;
  // ROS ingress: compare original integer stamps before converting once to
  // seconds. The legacy stamp_s fields are ignored on this entry point.
  SnapshotResult make_at_nanoseconds(const JointObservation &joints, const StampedPose &pose,
                                     const ModeFeedback &mode, std::int64_t application_ns) const;

private:
  SnapshotResult assemble(const JointObservation &joints, const Pose2d &pose,
                          const ModeFeedback &mode, double stamp_s) const;
  Config config_;
  std::array<std::string, 8> names_;
};
} // namespace swerve_mppi
