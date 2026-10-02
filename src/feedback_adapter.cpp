#include "swerve_mppi/feedback_adapter.hpp"
#include "swerve_mppi/model.hpp"
#include <cmath>

namespace swerve_mppi {
FeedbackAdapter::FeedbackAdapter(const Config &config, std::string prefix) : config_(config) {
  validate(config_);
  const std::array<std::string, 4> corners{"fl", "fr", "rl", "rr"};
  for (std::size_t i = 0; i < 4; ++i) {
    names_[i] = prefix + corners[i] + "_steering_joint";
    names_[i + 4] = prefix + corners[i] + "_wheel_joint";
  }
}
SnapshotResult FeedbackAdapter::make(const JointObservation &joints, const StampedPose &pose,
                                     const ModeFeedback &mode, double application_s) const {
  for (double t : {joints.stamp_s, pose.stamp_s, application_s})
    if (!std::isfinite(t) || t < 0)
      return {SnapshotError::InvalidTime, std::nullopt};
  // Equal source stamps, not a freshness allowance. A stale/raw observation is
  // not an execution-start state. Transport latency needs an explicit schedule.
  if (joints.stamp_s != pose.stamp_s || joints.stamp_s != application_s)
    return {SnapshotError::Unsynchronized, std::nullopt};
  if (joints.names.size() > 64 || joints.positions.size() != joints.names.size() ||
      joints.velocities.size() != joints.names.size())
    return {SnapshotError::MissingJoint, std::nullopt};
  VehicleState state;
  state.pose = pose.pose;
  state.stamp_s = joints.stamp_s;
  state.actual_mode = mode.actual_mode;
  state.mode_confirmed = mode.confirmed;
  state.mode_fault = mode.fault;
  state.mode_request_id = mode.request_id;
  state.time_in_mode_s = mode.time_in_mode_s;
  for (std::size_t i = 0; i < names_.size(); ++i) {
    std::size_t found = joints.names.size();
    for (std::size_t j = 0; j < joints.names.size(); ++j)
      if (joints.names[j] == names_[i]) {
        if (found != joints.names.size())
          return {SnapshotError::MissingJoint, std::nullopt};
        found = j;
      }
    if (found == joints.names.size())
      return {SnapshotError::MissingJoint, std::nullopt};
    if (i < 4)
      state.steering_angles[i] = joints.positions[found];
    else
      state.wheel_speeds[i - 4] = joints.velocities[found] * config_.wheel_radius_m;
  }
  state.velocity = Kinematics(config_).forward(state.wheel_speeds, state.steering_angles);
  if (check_model_feedback(state, config_).status != FeedbackStatus::Valid)
    return {SnapshotError::InvalidFeedback, std::nullopt};
  return {SnapshotError::None, state};
}
} // namespace swerve_mppi
