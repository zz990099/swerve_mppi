#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "swerve_mppi/common/config.hpp"
#include "swerve_mppi/common/types.hpp"

namespace swerve_mppi::current_chassis
{

constexpr std::uint8_t kDualAckermann = 0;
constexpr std::uint8_t kSpin = 1;
constexpr std::uint8_t kCrab = 2;
constexpr std::uint8_t kFaultPhase = 0;
constexpr std::uint8_t kBrakingPhase = 1;
constexpr std::uint8_t kAligningPhase = 2;
constexpr std::uint8_t kReadyPhase = 3;
constexpr std::uint8_t kNoFault = 0;
constexpr std::uint8_t kTransitionTimeoutFault = 5;

// ROS-independent DTOs mirror only the fields used by the current Python
// chassis messages. A future transport wrapper performs the mechanical copy.
struct TwistMessage
{
  double linear_x = 0;
  double linear_y = 0;
  double linear_z = 0;
  double angular_x = 0;
  double angular_y = 0;
  double angular_z = 0;
};

struct CommandMessage
{
  TimestampNs stamp_ns = kInvalidTimestamp;
  std::string frame_id;
  std::uint8_t mode = kDualAckermann;
  TwistMessage velocity;
  std::uint64_t request_id = 0;
  TwistMessage entry_velocity;
};

struct StateMessage
{
  TimestampNs stamp_ns = kInvalidTimestamp;
  std::string frame_id;
  std::uint64_t request_id = 0;
  std::uint8_t actual_mode = kDualAckermann;
  std::uint8_t requested_mode = kDualAckermann;
  std::uint8_t phase = kReadyPhase;
  bool confirmed = true;
  std::uint8_t fault = kNoFault;
  TwistMessage velocity;
  std::array<double, 4> steering_angles{};
  // Raw drive-joint angular velocity, rad/s.
  std::array<double, 4> wheel_speeds_radps{};
  TwistMessage accepted_entry_velocity;
  std::array<double, 4> accepted_steering{};
};

struct OdometryMessage
{
  TimestampNs stamp_ns = kInvalidTimestamp;
  std::string frame_id;
  std::string child_frame_id;
  Pose2d pose;
  TwistMessage velocity;
};

// Startup values copied from the current chassis configuration. Planner-only
// parameters deliberately do not appear here.
struct Parameters
{
  double wheelbase_m = 0.6;
  double track_m = 0.5;
  double wheel_radius_m = 0.1;
  double update_rate_hz = 100;
  double command_timeout_s = 0.5;
  double max_wheel_speed_radps = 20;
  double max_wheel_accel_radps2 = 40;
  double max_steer_rate_radps = 2.5;
  double steering_limit_rad = 1.5707963267948966;
  double max_linear_speed_mps = 0.8;
  double max_angular_speed_radps = 0.8;
  double max_linear_accel_mps2 = 0.9;
  double max_angular_accel_radps2 = 1.3;
  double drive_steering_limit_rad = 0.2;
  double steering_tolerance_rad = 0.05;
  double alignment_duration_s = 0.05;
  double transition_timeout_s = 5.0;
  double stopped_wheel_speed_radps = 0.05;
};

struct ParameterMismatch
{
  std::string name;
  double algorithm_value = 0;
  double chassis_value = 0;
};

struct CompatibilityResult
{
  std::vector<ParameterMismatch> mismatches;
  bool compatible() const { return mismatches.empty(); }
};

CompatibilityResult check_compatibility(const Config & config, const Parameters & parameters);

enum class AdapterError
{
  None,
  InvalidTime,
  Unsynchronized,
  Stale,
  Future,
  OutOfOrder,
  InvalidFrame,
  InvalidState,
  InconsistentState,
  InvalidTransition,
  NoObservation,
  NoAuthorizedCommand,
  StaleOutput,
  ExpiredOutput,
  InvalidPublication,
  InvalidApplication,
  RecoveryRequired
};

struct StateResult
{
  AdapterError error = AdapterError::InvalidState;
  std::optional<VehicleState> state;
};

// Evidence supplied by the transport/executor after publishing. The current
// chassis messages do not acknowledge an MPPI command ID, so the adapter never
// manufactures an exact application timestamp.
struct ApplicationEvidence
{
  std::uint64_t command_id = 0;
  TimestampNs earliest_application_ns = kInvalidTimestamp;
  TimestampNs latest_application_ns = kInvalidTimestamp;
};

struct CommandResult
{
  AdapterError error = AdapterError::NoAuthorizedCommand;
  std::optional<CommandMessage> message;
};

struct ApplicationResult
{
  AdapterError error = AdapterError::InvalidApplication;
  std::optional<CommandApplication> application;
};

class Adapter
{
public:
  Adapter(
    const Config & config, const Parameters & parameters, std::string body_frame = "base_footprint",
    std::string odom_frame = "odom");

  StateResult observe(
    const StateMessage & state, const OdometryMessage & odometry, TimestampNs planning_stamp_ns);
  CommandResult make_command(Output & output, TimestampNs publication_stamp_ns);
  ApplicationResult record_application(const ApplicationEvidence & evidence);

  // Chassis faults are cleared by a fresh zero request above the observed
  // request high-water mark. Repeated calls retry the same immutable request.
  CommandResult make_recovery_command(TimestampNs publication_stamp_ns);
  bool recovery_pending() const;

  // Full offline/process restart. This also forgets chassis request identity.
  void reset();

private:
  struct RecoveryRequest
  {
    std::uint64_t id = 0;
    DriveMode mode = DriveMode::DualAckermann;
    Twist2d entry_velocity;
  };
  struct PendingPublication
  {
    std::uint64_t command_id = 0;
    TimestampNs publication_stamp_ns = kInvalidTimestamp;
    TimestampNs valid_until_ns = kInvalidTimestamp;
  };

  Config config_;
  std::string body_frame_;
  std::string odom_frame_;
  TimestampNs last_state_stamp_ns_ = kInvalidTimestamp;
  TimestampNs last_odom_stamp_ns_ = kInvalidTimestamp;
  TimestampNs confirmed_since_ns_ = kInvalidTimestamp;
  TimestampNs last_publication_stamp_ns_ = kInvalidTimestamp;
  TimestampNs last_published_observation_stamp_ns_ = kInvalidTimestamp;
  std::uint64_t last_request_id_ = 0;
  std::uint64_t last_published_command_id_ = 0;
  std::optional<AcceptedModeRequest> accepted_request_;
  std::optional<StateMessage> last_wire_state_;
  std::optional<VehicleState> last_vehicle_;
  std::optional<RecoveryRequest> recovery_;
  std::optional<PendingPublication> pending_publication_;
};

}  // namespace swerve_mppi::current_chassis
