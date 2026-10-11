#pragma once
#include "planning/detail/prediction.hpp"
#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi::detail
{
class ModeManager
{
public:
  explicit ModeManager(const Config & config);
  void begin(DriveMode target_mode, const Control & entry_intent, const VehicleState & observed);
  bool active() const { return phase_ != TransitionPhase::Stable; }
  TransitionPhase phase() const { return phase_; }
  Prediction update(const VehicleState & observed);
  void reset();

private:
  Config config_;
  TransitionPhase phase_ = TransitionPhase::Stable;
  AcceptedModeRequest request_;
  std::optional<AcceptedModeRequest> accepted_request_;
  std::uint64_t last_request_id_ = 0;
  TimestampNs start_ns_ = 0;
  TimestampNs last_stamp_ns_ = kInvalidTimestamp;
};
}  // namespace swerve_mppi::detail
