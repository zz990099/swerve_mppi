#include "swerve_mppi/integration/adapter_contract.hpp"

#include <stdexcept>
#include <utility>

#include "swerve_mppi/execution/profile_runner.hpp"

namespace swerve_mppi
{
namespace
{
bool valid_identifier(const std::string & value)
{
  if (value.empty() || value.size() > 128) {
    return false;
  }
  for (unsigned char c : value) {
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
          c == '-' || c == '.' || c == '/' || c == ':')) {
      return false;
    }
  }
  return true;
}
bool valid_clock(const ClockIdentity & clock)
{
  return valid_identifier(clock.domain) && clock.epoch != 0;
}
bool same_clock(const ClockIdentity & a, const ClockIdentity & b)
{
  return a.domain == b.domain && a.epoch == b.epoch;
}
AdapterMetadata validated_metadata(const Config & config, const AdapterMetadata & metadata)
{
  validate_live_config(config);
  if (
    metadata.schema_version != 1 || !valid_identifier(metadata.world_frame) ||
    !valid_identifier(metadata.body_frame) || metadata.world_frame == metadata.body_frame ||
    !valid_clock(metadata.clock) || !metadata.timing ||
    (metadata.motion_policy != MotionEvidencePolicy::NominalEncoderOnly &&
     metadata.motion_policy != MotionEvidencePolicy::IndependentNominal)) {
    throw std::invalid_argument("invalid or incomplete adapter metadata");
  }
  // Reuse the actual consumers' timing/watchdog validation at startup.
  const TimingGuard timing(config, 1, *metadata.timing);
  const ProfileRunner profiles(config, metadata.watchdog_s);
  return metadata;
}
}  // namespace
AdapterContract::AdapterContract(std::string_view profile, const AdapterMetadata & metadata)
: config_(parse_resolved_config_profile(profile)), metadata_(validated_metadata(config_, metadata))
{
}
void require_adapter_compatible(const AdapterContract & local, const AdapterContract & peer)
{
  require_execution_compatible(local.config(), peer.config());
  const auto & a = local.metadata();
  const auto & b = peer.metadata();
  const auto mismatch = [](const char * field) {
    throw std::invalid_argument(std::string("adapter contract mismatch: ") + field);
  };
  if (a.schema_version != b.schema_version) {
    mismatch("schema_version");
  }
  if (a.world_frame != b.world_frame) {
    mismatch("world_frame");
  }
  if (a.body_frame != b.body_frame) {
    mismatch("body_frame");
  }
  if (!same_clock(a.clock, b.clock)) {
    mismatch("application_clock");
  }
  if (a.motion_policy != b.motion_policy) {
    mismatch("motion_policy");
  }
  if (
    a.timing->max_feedback_age_s != b.timing->max_feedback_age_s ||
    a.timing->max_command_age_s != b.timing->max_command_age_s ||
    a.timing->period_tolerance_ratio != b.timing->period_tolerance_ratio) {
    mismatch("timing_limits");
  }
  if (a.watchdog_s != b.watchdog_s) {
    mismatch("watchdog_s");
  }
}
SnapshotContractError check_snapshot_contract(
  const AdapterContract & contract, const SnapshotMetadata & snapshot)
{
  const auto & m = contract.metadata();
  if (!valid_clock(snapshot.application_clock) || snapshot.application_stamp_ns < 0) {
    return SnapshotContractError::InvalidMetadata;
  }
  if (!same_clock(snapshot.application_clock, m.clock)) {
    return SnapshotContractError::ClockMismatch;
  }
  const auto check = [&](const StampedFrame & source, const std::string & frame) {
    if (!valid_identifier(source.frame) || !valid_clock(source.clock) || source.stamp_ns < 0) {
      return SnapshotContractError::InvalidMetadata;
    }
    if (source.frame != frame) {
      return SnapshotContractError::FrameMismatch;
    }
    if (!same_clock(source.clock, m.clock)) {
      return SnapshotContractError::ClockMismatch;
    }
    if (source.stamp_ns != snapshot.encoders.stamp_ns) {
      return SnapshotContractError::Unsynchronized;
    }
    return SnapshotContractError::None;
  };
  for (auto pair :
       {std::make_pair(&snapshot.encoders, &m.body_frame),
        std::make_pair(&snapshot.pose, &m.world_frame),
        std::make_pair(&snapshot.mode, &m.body_frame),
        std::make_pair(&snapshot.context, &m.world_frame)}) {
    const auto error = check(*pair.first, *pair.second);
    if (error != SnapshotContractError::None) {
      return error;
    }
  }
  if (snapshot.body_observation) {
    const auto error = check(*snapshot.body_observation, m.body_frame);
    if (error != SnapshotContractError::None) {
      return error;
    }
  } else if (m.motion_policy == MotionEvidencePolicy::IndependentNominal) {
    return SnapshotContractError::MissingMotionObservation;
  }
  return snapshot.encoders.stamp_ns == snapshot.application_stamp_ns
           ? SnapshotContractError::None
           : SnapshotContractError::StateNotCurrent;
}
}  // namespace swerve_mppi
