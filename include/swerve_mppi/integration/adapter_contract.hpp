#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "swerve_mppi/common/config_profile.hpp"
#include "swerve_mppi/execution/timing.hpp"

namespace swerve_mppi
{
struct ClockIdentity
{
  std::string domain;
  std::uint64_t epoch = 0;  // Nonzero incarnation, changed after application-clock reset.
};
enum class MotionEvidencePolicy
{
  Unspecified,
  NominalEncoderOnly,  // Nominal development evidence, never independent acceptance.
  IndependentNominal   // Independent exact evidence plus all existing nominal gates.
};
struct AdapterMetadata
{
  std::uint32_t schema_version = 0;  // This API supports exactly version 1.
  std::string world_frame;
  std::string body_frame;
  ClockIdentity clock;
  MotionEvidencePolicy motion_policy = MotionEvidencePolicy::Unspecified;
  std::optional<TimingLimits> timing;
  double watchdog_s = 0;
};
// Startup-only immutable contract. Requires a complete resolved profile, valid
// live budget and explicit metadata. No transport, hash-based agreement, runtime
// arming, TF lookup or automatic configuration propagation is provided.
class AdapterContract
{
public:
  AdapterContract(std::string_view resolved_profile, const AdapterMetadata & metadata);
  const Config & config() const { return config_; }
  const AdapterMetadata & metadata() const { return metadata_; }

private:
  const Config config_;
  const AdapterMetadata metadata_;
};
void require_adapter_compatible(const AdapterContract & local, const AdapterContract & peer);
struct StampedFrame
{
  std::string frame;
  ClockIdentity clock;
  std::int64_t stamp_ns = -1;
};
struct SnapshotMetadata
{
  StampedFrame pose;
  StampedFrame encoders;
  StampedFrame mode;
  // Current assembled task/obstacle context in the same world frame. Static
  // geometry may persist; stamp names the context snapshot, not a fresh sensor.
  StampedFrame context;
  std::optional<StampedFrame> body_observation;
  ClockIdentity application_clock;
  std::int64_t application_stamp_ns = -1;
};
enum class SnapshotContractError
{
  None,
  InvalidMetadata,
  FrameMismatch,
  ClockMismatch,
  Unsynchronized,
  StateNotCurrent,
  MissingMotionObservation
};
// Exact frame/clock/source-stamp checks before conversion. Metadata must be
// bound to the actual decoded values. This cannot verify that association or
// transform frames, establish physical validity, assess motion residuals, check
// cadence/freshness, or replace FeedbackAdapter/MotionObserver/TimedExecutor.
SnapshotContractError check_snapshot_contract(
  const AdapterContract & contract, const SnapshotMetadata & snapshot);
}  // namespace swerve_mppi
