#include "swerve_mppi/adapter/current_chassis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi::current_chassis
{
namespace
{
constexpr double kNumericalTolerance = 1e-9;

bool finite(double value) { return std::isfinite(value); }

bool close(double a, double b)
{
  return std::abs(a - b) <= kNumericalTolerance * std::max({1.0, std::abs(a), std::abs(b)});
}

bool zero(double value) { return value == 0; }

bool planar(const TwistMessage & twist)
{
  return finite(twist.linear_x) && finite(twist.linear_y) && finite(twist.linear_z) &&
         finite(twist.angular_x) && finite(twist.angular_y) && finite(twist.angular_z) &&
         zero(twist.linear_z) && zero(twist.angular_x) && zero(twist.angular_y);
}

Twist2d core_twist(const TwistMessage & twist)
{
  return {twist.linear_x, twist.linear_y, twist.angular_z};
}

TwistMessage wire_twist(const Twist2d & twist)
{
  TwistMessage out;
  out.linear_x = twist.vx;
  out.linear_y = twist.vy;
  out.angular_z = twist.wz;
  return out;
}

std::optional<DriveMode> core_mode(std::uint8_t mode)
{
  switch (mode) {
    case kDualAckermann:
      return DriveMode::DualAckermann;
    case kSpin:
      return DriveMode::Spin;
    case kCrab:
      return DriveMode::Crab;
    default:
      return std::nullopt;
  }
}

std::uint8_t wire_mode(DriveMode mode)
{
  switch (mode) {
    case DriveMode::DualAckermann:
      return kDualAckermann;
    case DriveMode::Spin:
      return kSpin;
    case DriveMode::Crab:
      return kCrab;
  }
  return std::numeric_limits<std::uint8_t>::max();
}

bool permitted(DriveMode mode, const Twist2d & velocity)
{
  if (!finite(velocity.vx) || !finite(velocity.vy) || !finite(velocity.wz)) return false;
  switch (mode) {
    case DriveMode::DualAckermann:
      return zero(velocity.vy) && (!zero(velocity.vx) || zero(velocity.wz));
    case DriveMode::Spin:
      return zero(velocity.vx) && zero(velocity.vy);
    case DriveMode::Crab:
      return zero(velocity.wz);
  }
  return false;
}

bool stopped(const Twist2d & velocity)
{
  return zero(velocity.vx) && zero(velocity.vy) && zero(velocity.wz);
}

bool same_twist(const Twist2d & a, const Twist2d & b)
{
  return a.vx == b.vx && a.vy == b.vy && a.wz == b.wz;
}

bool same_request(const AcceptedModeRequest & a, const AcceptedModeRequest & b)
{
  return a.id == b.id && a.mode == b.mode && same_twist(a.entry_velocity, b.entry_velocity) &&
         a.steering_targets == b.steering_targets;
}

bool finite_array(const std::array<double, 4> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return finite(value); });
}

bool within_steering_limit(const std::array<double, 4> & values, double limit)
{
  return finite_array(values) && std::all_of(values.begin(), values.end(), [limit](double value) {
           return std::abs(value) <= limit;
         });
}

bool consistent_twist(const Twist2d & expected, const TwistMessage & message)
{
  return planar(message) && close(expected.vx, message.linear_x) &&
         close(expected.vy, message.linear_y) && close(expected.wz, message.angular_z);
}

void compare(CompatibilityResult & result, const char * name, double algorithm, double chassis)
{
  if (!finite(algorithm) || !finite(chassis) || !close(algorithm, chassis)) {
    result.mismatches.push_back({name, algorithm, chassis});
  }
}
}  // namespace

CompatibilityResult check_compatibility(const Config & config, const Parameters & parameters)
{
  validate(config);
  CompatibilityResult result;
  compare(result, "wheelbase", config.wheelbase_m, parameters.wheelbase_m);
  compare(result, "track_width", config.track_m, parameters.track_m);
  compare(result, "wheel_radius", config.wheel_radius_m, parameters.wheel_radius_m);
  const double period = finite(parameters.update_rate_hz) && parameters.update_rate_hz > 0
                          ? 1.0 / parameters.update_rate_hz
                          : std::numeric_limits<double>::quiet_NaN();
  compare(result, "update_rate", config.chassis_period_s, period);
  compare(
    result, "max_wheel_speed", config.max_wheel_speed_mps,
    parameters.max_wheel_speed_radps * parameters.wheel_radius_m);
  compare(
    result, "max_wheel_acceleration", config.max_wheel_accel_mps2,
    parameters.max_wheel_accel_radps2 * parameters.wheel_radius_m);
  compare(
    result, "max_steering_rate", config.max_steer_rate_radps, parameters.max_steer_rate_radps);
  compare(result, "steering_limit", config.steering_limit_rad, parameters.steering_limit_rad);
  compare(
    result, "max_linear_speed", config.chassis_max_linear_speed_mps,
    parameters.max_linear_speed_mps);
  compare(
    result, "max_angular_speed", config.chassis_max_angular_speed_radps,
    parameters.max_angular_speed_radps);
  compare(
    result, "max_linear_acceleration", config.max_linear_accel_mps2,
    parameters.max_linear_accel_mps2);
  compare(
    result, "max_angular_acceleration", config.max_angular_accel_radps2,
    parameters.max_angular_accel_radps2);
  compare(
    result, "drive_steering_limit", config.drive_steering_limit_rad,
    parameters.drive_steering_limit_rad);
  compare(
    result, "steering_alignment_tolerance", config.steering_tolerance_rad,
    parameters.steering_tolerance_rad);
  compare(
    result, "steering_alignment_duration", config.alignment_min_s, parameters.alignment_duration_s);
  compare(
    result, "mode_switch_timeout", config.confirmation_timeout_s, parameters.transition_timeout_s);
  compare(
    result, "stopped_wheel_speed", config.stopped_wheel_speed_mps,
    parameters.stopped_wheel_speed_radps * parameters.wheel_radius_m);
  if (
    !finite(parameters.command_timeout_s) || parameters.command_timeout_s <= 0 ||
    config.command_lifetime_s > parameters.command_timeout_s + kNumericalTolerance) {
    result.mismatches.push_back(
      {"cmd_timeout", config.command_lifetime_s, parameters.command_timeout_s});
  }
  return result;
}

Adapter::Adapter(
  const Config & config, const Parameters & parameters, std::string body_frame,
  std::string odom_frame)
: config_(config), body_frame_(std::move(body_frame)), odom_frame_(std::move(odom_frame))
{
  const auto compatibility = check_compatibility(config_, parameters);
  if (!compatibility.compatible()) {
    throw std::invalid_argument(
      "current chassis parameter mismatch: " + compatibility.mismatches.front().name);
  }
  if (body_frame_.empty() || odom_frame_.empty() || body_frame_ == odom_frame_) {
    throw std::invalid_argument("current chassis frames must be nonempty and distinct");
  }
}

StateResult Adapter::observe(
  const StateMessage & state, const OdometryMessage & odometry, TimestampNs planning_stamp_ns)
{
  if (state.stamp_ns < 0 || odometry.stamp_ns < 0 || planning_stamp_ns < 0) {
    return {AdapterError::InvalidTime, std::nullopt};
  }
  if (
    state.frame_id != body_frame_ || odometry.frame_id != odom_frame_ ||
    odometry.child_frame_id != body_frame_) {
    return {AdapterError::InvalidFrame, std::nullopt};
  }
  if (
    (last_state_stamp_ns_ >= 0 && state.stamp_ns <= last_state_stamp_ns_) ||
    (last_odom_stamp_ns_ >= 0 && odometry.stamp_ns <= last_odom_stamp_ns_)) {
    return {AdapterError::OutOfOrder, std::nullopt};
  }
  const auto oldest = std::min(state.stamp_ns, odometry.stamp_ns);
  const auto newest = std::max(state.stamp_ns, odometry.stamp_ns);
  const auto pairing = duration_nanoseconds(config_.observation_pairing_tolerance_s);
  const auto future = duration_nanoseconds(config_.future_observation_tolerance_s);
  const auto age = duration_nanoseconds(config_.max_observation_age_s);
  if (!pairing || newest - oldest > *pairing) {
    return {AdapterError::Unsynchronized, std::nullopt};
  }
  if (!future || (newest > planning_stamp_ns && newest - planning_stamp_ns > *future)) {
    return {AdapterError::Future, std::nullopt};
  }
  if (!age || (planning_stamp_ns > newest && planning_stamp_ns - newest > *age)) {
    return {AdapterError::Stale, std::nullopt};
  }
  const auto actual_mode = core_mode(state.actual_mode);
  const auto requested_mode = core_mode(state.requested_mode);
  if (
    !actual_mode || !requested_mode || state.phase > kReadyPhase ||
    state.fault > kTransitionTimeoutFault || !planar(state.velocity) ||
    !planar(state.accepted_entry_velocity) || !planar(odometry.velocity) ||
    !within_steering_limit(state.steering_angles, config_.steering_limit_rad) ||
    !within_steering_limit(state.accepted_steering, config_.steering_limit_rad) ||
    !finite(odometry.pose.x) || !finite(odometry.pose.y) || !finite(odometry.pose.yaw)) {
    return {AdapterError::InvalidState, std::nullopt};
  }
  const bool ready = state.phase == kReadyPhase && state.fault == kNoFault;
  const bool faulted = state.phase == kFaultPhase && state.fault != kNoFault;
  if (
    state.confirmed != ready || (state.fault == kNoFault && state.phase == kFaultPhase) ||
    (state.fault != kNoFault && !faulted) || (state.confirmed && *actual_mode != *requested_mode) ||
    state.request_id < last_request_id_) {
    return {AdapterError::InvalidTransition, std::nullopt};
  }
  if (state.request_id == 0) {
    if (
      *actual_mode != DriveMode::DualAckermann || *requested_mode != DriveMode::DualAckermann ||
      !stopped(core_twist(state.accepted_entry_velocity)) ||
      state.accepted_steering != std::array<double, 4>{}) {
      return {AdapterError::InvalidTransition, std::nullopt};
    }
  }
  std::optional<AcceptedModeRequest> receipt;
  if (state.request_id > 0) {
    const auto entry = core_twist(state.accepted_entry_velocity);
    if (!permitted(*requested_mode, entry)) {
      return {AdapterError::InvalidTransition, std::nullopt};
    }
    receipt =
      AcceptedModeRequest{state.request_id, *requested_mode, state.accepted_steering, entry};
    if (
      state.request_id == last_request_id_ && accepted_request_ &&
      !same_request(*receipt, *accepted_request_)) {
      return {AdapterError::InvalidTransition, std::nullopt};
    }
    if (recovery_) {
      if (
        state.request_id > recovery_->id ||
        (state.request_id == recovery_->id &&
         (receipt->mode != recovery_->mode ||
          !same_twist(receipt->entry_velocity, recovery_->entry_velocity)))) {
        return {AdapterError::InvalidTransition, std::nullopt};
      }
    }
  }

  VehicleState vehicle;
  vehicle.pose = odometry.pose;
  vehicle.steering_angles = state.steering_angles;
  for (std::size_t i = 0; i < vehicle.wheel_speeds.size(); ++i) {
    if (!finite(state.wheel_speeds_radps[i])) {
      return {AdapterError::InvalidState, std::nullopt};
    }
    vehicle.wheel_speeds[i] = state.wheel_speeds_radps[i] * config_.wheel_radius_m;
    if (std::abs(vehicle.wheel_speeds[i]) > config_.max_wheel_speed_mps + kNumericalTolerance) {
      return {AdapterError::InvalidState, std::nullopt};
    }
  }
  vehicle.velocity = Kinematics(config_).forward(vehicle.wheel_speeds, vehicle.steering_angles);
  if (
    !consistent_twist(vehicle.velocity, state.velocity) ||
    !consistent_twist(vehicle.velocity, odometry.velocity)) {
    return {AdapterError::InconsistentState, std::nullopt};
  }
  vehicle.actual_mode = *actual_mode;
  vehicle.mode_confirmed = state.confirmed;
  vehicle.mode_fault = state.fault != kNoFault;
  vehicle.mode_request_id = state.request_id;
  vehicle.accepted_mode_request = receipt;
  vehicle.stamp_ns = newest;

  const bool continuous_confirmation =
    last_wire_state_ && last_wire_state_->confirmed && state.confirmed &&
    last_wire_state_->actual_mode == state.actual_mode &&
    last_wire_state_->request_id == state.request_id && confirmed_since_ns_ >= 0;
  if (state.confirmed) {
    if (!continuous_confirmation) confirmed_since_ns_ = newest;
    vehicle.time_in_mode_s = duration_seconds(newest - confirmed_since_ns_);
  } else {
    confirmed_since_ns_ = kInvalidTimestamp;
    vehicle.time_in_mode_s = 0;
  }

  if (
    recovery_ && state.request_id == recovery_->id && state.confirmed && state.fault == kNoFault &&
    *actual_mode == recovery_->mode) {
    recovery_.reset();
  }
  last_state_stamp_ns_ = state.stamp_ns;
  last_odom_stamp_ns_ = odometry.stamp_ns;
  last_request_id_ = state.request_id;
  accepted_request_ = receipt;
  last_wire_state_ = state;
  last_vehicle_ = vehicle;
  return {AdapterError::None, vehicle};
}

CommandResult Adapter::make_command(Output & output, TimestampNs publication_stamp_ns)
{
  if (!last_vehicle_ || !last_wire_state_) {
    return {AdapterError::NoObservation, std::nullopt};
  }
  if (recovery_ || last_vehicle_->mode_fault) {
    return {AdapterError::RecoveryRequired, std::nullopt};
  }
  if (!output.command || output.command_id == 0) {
    return {AdapterError::NoAuthorizedCommand, std::nullopt};
  }
  if (
    output.observation_stamp_ns != last_vehicle_->stamp_ns ||
    output.observation_stamp_ns <= last_published_observation_stamp_ns_ ||
    output.command_id <= last_published_command_id_) {
    return {AdapterError::StaleOutput, std::nullopt};
  }
  if (
    publication_stamp_ns < 0 || output.computed_stamp_ns < output.observation_stamp_ns ||
    publication_stamp_ns < output.computed_stamp_ns ||
    publication_stamp_ns < last_publication_stamp_ns_ || output.published_stamp_ns >= 0) {
    return {AdapterError::InvalidPublication, std::nullopt};
  }
  if (publication_stamp_ns > output.valid_until_ns) {
    return {AdapterError::ExpiredOutput, std::nullopt};
  }

  const auto & command = *output.command;
  if (!permitted(command.mode, command.target_velocity)) {
    return {AdapterError::NoAuthorizedCommand, std::nullopt};
  }
  CommandMessage message;
  message.stamp_ns = publication_stamp_ns;
  message.frame_id = body_frame_;
  message.velocity = wire_twist(command.target_velocity);
  if (command.mode_request) {
    const auto & request = *command.mode_request;
    if (
      request.id == 0 || request.id < last_request_id_ || request.mode != command.mode ||
      !stopped(command.target_velocity) || !permitted(request.mode, request.entry_velocity) ||
      (request.id > last_request_id_ && !last_vehicle_->mode_confirmed)) {
      return {AdapterError::InvalidTransition, std::nullopt};
    }
    if (
      request.id == last_request_id_ &&
      (!accepted_request_ || accepted_request_->mode != request.mode ||
       !same_twist(accepted_request_->entry_velocity, request.entry_velocity))) {
      return {AdapterError::InvalidTransition, std::nullopt};
    }
    message.mode = wire_mode(request.mode);
    message.request_id = request.id;
    message.entry_velocity = wire_twist(request.entry_velocity);
  } else {
    if (
      command.mode != last_vehicle_->actual_mode ||
      (!stopped(command.target_velocity) && !last_vehicle_->mode_confirmed) ||
      (accepted_request_ && accepted_request_->mode != last_vehicle_->actual_mode)) {
      return {AdapterError::InvalidTransition, std::nullopt};
    }
    message.request_id = last_request_id_;
    if (accepted_request_) {
      message.mode = wire_mode(accepted_request_->mode);
      message.entry_velocity = wire_twist(accepted_request_->entry_velocity);
    } else {
      message.mode = wire_mode(command.mode);
    }
  }
  if (!set_publication_stamp(output, publication_stamp_ns)) {
    return {AdapterError::InvalidPublication, std::nullopt};
  }
  last_published_command_id_ = output.command_id;
  last_publication_stamp_ns_ = publication_stamp_ns;
  last_published_observation_stamp_ns_ = output.observation_stamp_ns;
  pending_publication_ =
    PendingPublication{output.command_id, publication_stamp_ns, output.valid_until_ns};
  return {AdapterError::None, message};
}

ApplicationResult Adapter::record_application(const ApplicationEvidence & evidence)
{
  if (!pending_publication_ || evidence.command_id != pending_publication_->command_id) {
    return {AdapterError::InvalidApplication, std::nullopt};
  }
  const auto uncertainty = duration_nanoseconds(config_.max_command_application_uncertainty_s);
  if (
    evidence.earliest_application_ns < pending_publication_->publication_stamp_ns ||
    evidence.latest_application_ns < evidence.earliest_application_ns || !uncertainty ||
    evidence.latest_application_ns - evidence.earliest_application_ns > *uncertainty ||
    evidence.latest_application_ns > pending_publication_->valid_until_ns) {
    return {AdapterError::InvalidApplication, std::nullopt};
  }
  const CommandApplication application{
    pending_publication_->command_id, pending_publication_->publication_stamp_ns,
    evidence.earliest_application_ns, evidence.latest_application_ns};
  pending_publication_.reset();
  return {AdapterError::None, application};
}

CommandResult Adapter::make_recovery_command(TimestampNs publication_stamp_ns)
{
  if (!last_vehicle_ || !last_wire_state_) {
    return {AdapterError::NoObservation, std::nullopt};
  }
  if (!last_vehicle_->mode_fault && !recovery_) {
    return {AdapterError::NoAuthorizedCommand, std::nullopt};
  }
  if (
    publication_stamp_ns < last_vehicle_->stamp_ns ||
    publication_stamp_ns < last_publication_stamp_ns_) {
    return {AdapterError::InvalidTime, std::nullopt};
  }
  if (!recovery_) {
    if (last_request_id_ == std::numeric_limits<std::uint64_t>::max()) {
      return {AdapterError::InvalidTransition, std::nullopt};
    }
    RecoveryRequest request;
    request.id = last_request_id_ + 1;
    request.mode = last_vehicle_->actual_mode;
    if (accepted_request_ && accepted_request_->mode == request.mode) {
      request.entry_velocity = accepted_request_->entry_velocity;
    }
    recovery_ = request;
  }
  pending_publication_.reset();
  CommandMessage message;
  message.stamp_ns = publication_stamp_ns;
  message.frame_id = body_frame_;
  message.mode = wire_mode(recovery_->mode);
  message.request_id = recovery_->id;
  message.entry_velocity = wire_twist(recovery_->entry_velocity);
  last_publication_stamp_ns_ = publication_stamp_ns;
  return {AdapterError::None, message};
}

bool Adapter::recovery_pending() const { return recovery_.has_value(); }

void Adapter::reset()
{
  last_state_stamp_ns_ = kInvalidTimestamp;
  last_odom_stamp_ns_ = kInvalidTimestamp;
  confirmed_since_ns_ = kInvalidTimestamp;
  last_publication_stamp_ns_ = kInvalidTimestamp;
  last_published_observation_stamp_ns_ = kInvalidTimestamp;
  last_request_id_ = 0;
  last_published_command_id_ = 0;
  accepted_request_.reset();
  last_wire_state_.reset();
  last_vehicle_.reset();
  recovery_.reset();
  pending_publication_.reset();
}

}  // namespace swerve_mppi::current_chassis
