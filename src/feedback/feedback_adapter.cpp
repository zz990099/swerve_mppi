#include "swerve_mppi/feedback/feedback_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi
{
FeedbackAdapter::FeedbackAdapter(const Config & config, std::string prefix) : config_(config)
{
  validate(config_);
  const std::array<std::string, 4> corners{"fl", "fr", "rl", "rr"};
  for (std::size_t i = 0; i < 4; ++i) {
    names_[i] = prefix + corners[i] + "_steering_joint";
    names_[i + 4] = prefix + corners[i] + "_wheel_joint";
  }
}
SnapshotResult FeedbackAdapter::make(
  const JointObservation & joints, const StampedPose & pose, const ModeFeedback & mode,
  TimestampNs planning_stamp_ns) const
{
  if (joints.stamp_ns < 0 || pose.stamp_ns < 0 || mode.stamp_ns < 0 || planning_stamp_ns < 0) {
    return {SnapshotError::InvalidTime, std::nullopt};
  }
  const auto oldest = std::min({joints.stamp_ns, pose.stamp_ns, mode.stamp_ns});
  const auto newest = std::max({joints.stamp_ns, pose.stamp_ns, mode.stamp_ns});
  const auto pairing = duration_nanoseconds(config_.observation_pairing_tolerance_s);
  const auto age = duration_nanoseconds(config_.max_observation_age_s);
  const auto future = duration_nanoseconds(config_.future_observation_tolerance_s);
  if (!pairing || newest - oldest > *pairing) {
    return {SnapshotError::Unsynchronized, std::nullopt};
  }
  if (!future || (newest > planning_stamp_ns && newest - planning_stamp_ns > *future)) {
    return {SnapshotError::Future, std::nullopt};
  }
  if (!age || (planning_stamp_ns > newest && planning_stamp_ns - newest > *age)) {
    return {SnapshotError::Stale, std::nullopt};
  }
  return assemble(joints, pose.pose, mode, newest);
}
SnapshotResult FeedbackAdapter::assemble(
  const JointObservation & joints, const Pose2d & pose, const ModeFeedback & mode,
  TimestampNs stamp_ns) const
{
  if (
    joints.names.size() > 64 || joints.positions.size() != joints.names.size() ||
    joints.velocities.size() != joints.names.size()) {
    return {SnapshotError::MissingJoint, std::nullopt};
  }
  VehicleState state;
  state.pose = pose;
  state.stamp_ns = stamp_ns;
  state.actual_mode = mode.actual_mode;
  state.mode_confirmed = mode.confirmed;
  state.mode_fault = mode.fault;
  state.mode_request_id = mode.request_id;
  state.accepted_mode_request = mode.accepted_mode_request;
  state.time_in_mode_s = mode.time_in_mode_s;
  for (std::size_t i = 0; i < names_.size(); ++i) {
    std::size_t found = joints.names.size();
    for (std::size_t j = 0; j < joints.names.size(); ++j) {
      if (joints.names[j] == names_[i]) {
        if (found != joints.names.size()) {
          return {SnapshotError::MissingJoint, std::nullopt};
        }
        found = j;
      }
    }
    if (found == joints.names.size()) {
      return {SnapshotError::MissingJoint, std::nullopt};
    }
    if (i < 4) {
      state.steering_angles[i] = joints.positions[found];
    } else {
      state.wheel_speeds[i - 4] = joints.velocities[found] * config_.wheel_radius_m;
    }
  }
  state.velocity = Kinematics(config_).forward(state.wheel_speeds, state.steering_angles);
  if (check_model_feedback(state, config_).status != FeedbackStatus::Valid) {
    return {SnapshotError::InvalidFeedback, std::nullopt};
  }
  return {SnapshotError::None, state};
}
}  // namespace swerve_mppi
