#pragma once

#include <chrono>
#include <cmath>

#include "swerve_mppi/execution/profile_runner.hpp"
#include "swerve_mppi/feedback/feedback_adapter.hpp"
#include "swerve_mppi/feedback/motion_observer.hpp"
#include "swerve_mppi/integration/adapter_contract.hpp"

namespace swerve_mppi::test
{
// Transport-free owner example used only by offline contract tests. It has no
// threads, ROS, TF, actuator servo or physical stop certificate. A real owner
// must bind metadata to received payloads, serialize callbacks, and implement
// independent endpoint stop/acknowledgement and queue draining.
struct AdapterPacket
{
  JointObservation joints;
  StampedPose pose;
  ModeFeedback mode;
  ControllerInput context;
  SnapshotMetadata metadata;
  std::optional<MotionObservation> body;
};
class AdapterOwnerFixture
{
public:
  AdapterOwnerFixture(const AdapterContract & local, const AdapterContract & peer)
  : contract_(local),
    adapter_(local.config(), "bot_"),
    observer_(local.config()),
    executor_(local.config(), session_, DriveMode::DualAckermann, *local.metadata().timing),
    runner_(local.config(), local.metadata().watchdog_s)
  {
    require_adapter_compatible(local, peer);
    runner_.cancel();  // Startup is unarmed; an empty profile cannot retain Drive.
  }
  std::optional<ControllerInput> decode(const AdapterPacket & p, std::int64_t current_ns) const
  {
    if (
      p.context.reference_path.empty() ||
      p.context.reference_path.size() > contract_.config().max_path_points ||
      p.context.obstacles.size() > contract_.config().max_obstacles ||
      p.metadata.application_stamp_ns != current_ns ||
      check_snapshot_contract(contract_, p.metadata) != SnapshotContractError::None ||
      p.joints.stamp_ns != p.metadata.encoders.stamp_ns ||
      p.pose.stamp_ns != p.metadata.pose.stamp_ns ||
      p.body.has_value() != p.metadata.body_observation.has_value() ||
      (p.body && p.body->stamp_ns != p.metadata.body_observation->stamp_ns)) {
      return {};
    }
    const auto snapshot =
      adapter_.make_at_nanoseconds(p.joints, p.pose, p.mode, p.metadata.application_stamp_ns);
    if (!snapshot.state) {
      return {};
    }
    // If an independent channel is present, disagreement must not be ignored
    // even in the explicitly selected nominal encoder development policy.
    if (p.body || contract_.metadata().motion_policy == MotionEvidencePolicy::IndependentNominal) {
      if (
        observer_.assess(*snapshot.state, p.joints.stamp_ns, p.body).status !=
        MotionStatus::NominalAgreement) {
        return {};
      }
    }
    auto context = p.context;
    context.vehicle = *snapshot.state;  // Never trust a redundant supplied vehicle.
    return context;
  }
  bool observe_stopped(const AdapterPacket & p, std::int64_t current_ns)
  {
    const auto input = decode(p, current_ns);
    if (
      !input || !stopped(p, input->vehicle) || input->vehicle.mode_fault ||
      !input->vehicle.mode_confirmed) {
      clear_stop_evidence();
      return false;
    }
    const auto stamp = p.joints.stamp_ns;
    if (last_stop_ns_ >= 0) {
      const double interval =
        std::chrono::duration<double>(std::chrono::nanoseconds(stamp - last_stop_ns_)).count();
      const double tolerance = contract_.metadata().timing->period_tolerance_ratio;
      if (
        stamp <= last_stop_ns_ || std::abs(interval - contract_.config().dt_s) >
                                    tolerance * contract_.config().dt_s + 1e-12) {
        clear_stop_evidence();
        return false;
      }
    }
    if (first_stop_ns_ < 0) {
      first_stop_ns_ = stamp;
    }
    last_stop_ns_ = stamp;
    const double dwell =
      std::chrono::duration<double>(std::chrono::nanoseconds(last_stop_ns_ - first_stop_ns_))
        .count();
    return dwell >= contract_.config().goal_settle_time_s;
  }
  bool recover(
    const AdapterPacket & p, std::uint64_t renewed_session, bool queues_drained,
    bool stopped_endpoint_ack, std::int64_t current_ns)
  {
    // These flags model external acknowledgements in this fixture. They are
    // not a replacement for a physical stop or a transport drain protocol.
    const auto current = decode(p, current_ns);
    if (
      !fault_ || !current || renewed_session <= session_ || !queues_drained ||
      !stopped_endpoint_ack || last_stop_ns_ != p.joints.stamp_ns || first_stop_ns_ < 0 ||
      !stopped(p, current->vehicle) || current->vehicle.mode_fault ||
      !current->vehicle.mode_confirmed ||
      std::chrono::duration<double>(std::chrono::nanoseconds(last_stop_ns_ - first_stop_ns_))
          .count() < contract_.config().goal_settle_time_s) {
      trip();
      return false;
    }
    executor_.reset(current->vehicle, renewed_session);
    runner_.reset(current->vehicle);
    session_ = renewed_session;
    fault_ = false;
    clear_stop_evidence();
    return true;
  }
  std::optional<TimedExecutionResult> apply(
    const AdapterPacket & p, const std::optional<CommandEnvelope> & command, double wall_s,
    std::int64_t current_ns)
  {
    const auto current = decode(p, current_ns);
    if (fault_ || !current) {
      trip();
      return {};
    }
    const auto result = executor_.update(command, *current, current->vehicle.stamp_s);
    if (!runner_.install(result, current->vehicle.stamp_s, wall_s)) {
      trip();
    }
    return result;
  }
  std::optional<JointTargets> sample(double now_s, double wall_s)
  {
    if (fault_) {
      return {};
    }
    auto targets = runner_.sample(now_s, wall_s);
    if (!targets) {
      trip();
    }
    return targets;
  }
  void trip()
  {
    runner_.cancel();  // Revoke old Drive even before another execution tick.
    fault_ = true;
    ++stop_requests_;  // Abstract external emergency-stop channel, not plant braking.
    clear_stop_evidence();
  }
  bool fault() const { return fault_; }
  std::uint64_t session() const { return session_; }
  std::size_t stop_requests() const { return stop_requests_; }

private:
  bool stopped(const AdapterPacket & p, const VehicleState & state) const
  {
    if (!is_stopped(state, contract_.config())) {
      return false;
    }
    if (p.body) {
      const auto a = observer_.assess(state, p.joints.stamp_ns, p.body);
      return a.status == MotionStatus::NominalAgreement && a.body_stationary &&
             a.encoder_stationary;
    }
    return contract_.metadata().motion_policy == MotionEvidencePolicy::NominalEncoderOnly;
  }
  void clear_stop_evidence() { first_stop_ns_ = last_stop_ns_ = -1; }
  AdapterContract contract_;
  FeedbackAdapter adapter_;
  MotionObserver observer_;
  std::uint64_t session_ = 1;
  TimedExecutor executor_;
  ProfileRunner runner_;
  bool fault_ = true;
  std::size_t stop_requests_ = 0;
  std::int64_t first_stop_ns_ = -1;
  std::int64_t last_stop_ns_ = -1;
};
inline AdapterMetadata adapter_metadata(
  MotionEvidencePolicy policy = MotionEvidencePolicy::IndependentNominal)
{
  return {1, "odom", "base_link", {"test_sim", 1}, policy, TimingLimits{}, .5};
}
inline AdapterPacket stopped_packet(std::int64_t stamp = 1000000000)
{
  AdapterPacket p;
  p.joints.stamp_ns = p.pose.stamp_ns = stamp;
  p.context.reference_path = {{0, 0, 0}, {1, 0, 0}};
  p.mode.time_in_mode_s = 2;
  p.metadata.application_clock = {"test_sim", 1};
  p.metadata.application_stamp_ns = stamp;
  p.metadata.pose = {"odom", {"test_sim", 1}, stamp};
  p.metadata.context = p.metadata.pose;
  p.metadata.encoders = {"base_link", {"test_sim", 1}, stamp};
  p.metadata.mode = p.metadata.encoders;
  p.metadata.body_observation = p.metadata.encoders;
  p.body = MotionObservation{{}, stamp, MotionSource::IndependentBody};
  for (const char * corner : {"fl", "fr", "rl", "rr"}) {
    for (const char * joint : {"_wheel_joint", "_steering_joint"}) {
      p.joints.names.push_back(std::string("bot_") + corner + joint);
      p.joints.positions.push_back(0);
      p.joints.velocities.push_back(0);
    }
  }
  return p;
}
inline CommandEnvelope adapter_command(
  const AdapterPacket & p, std::uint64_t session, std::uint64_t sequence, bool request_mode = false)
{
  const double now =
    std::chrono::duration<double>(std::chrono::nanoseconds(p.joints.stamp_ns)).count();
  Output output;
  output.command = ChassisCommand{DriveMode::DualAckermann, {.3, 0, 0}, {}};
  if (request_mode) {
    output.command =
      ChassisCommand{DriveMode::Crab, {}, ModeRequest{7, DriveMode::Crab, {0, .3, 0}}};
  }
  return {session, sequence, now, output, now, now, now + .025, CommandTask::capture(p.context)};
}
}  // namespace swerve_mppi::test
