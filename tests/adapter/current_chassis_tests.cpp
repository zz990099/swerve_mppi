#include <cmath>
#include <iostream>
#include <stdexcept>

#include "swerve_mppi/adapter/current_chassis.hpp"
#include "swerve_mppi/model/model.hpp"

using namespace swerve_mppi;
using namespace swerve_mppi::current_chassis;

namespace
{
void check(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

TwistMessage wire(const Twist2d & twist)
{
  TwistMessage out;
  out.linear_x = twist.vx;
  out.linear_y = twist.vy;
  out.angular_z = twist.wz;
  return out;
}

void update_twist(StateMessage & state, OdometryMessage & odometry, const Config & config)
{
  std::array<double, 4> rolling{};
  for (std::size_t i = 0; i < rolling.size(); ++i) {
    rolling[i] = state.wheel_speeds_radps[i] * config.wheel_radius_m;
  }
  state.velocity = wire(Kinematics(config).forward(rolling, state.steering_angles));
  odometry.velocity = state.velocity;
}

void stamp(StateMessage & state, OdometryMessage & odometry, TimestampNs value)
{
  state.stamp_ns = value;
  odometry.stamp_ns = value;
}

struct Messages
{
  StateMessage state;
  OdometryMessage odometry;
};

Messages startup(const Config & config, TimestampNs stamp_ns = 1000000000)
{
  Messages messages;
  messages.state.frame_id = "base_footprint";
  messages.odometry.frame_id = "odom";
  messages.odometry.child_frame_id = "base_footprint";
  messages.state.wheel_speeds_radps.fill(1.0);
  stamp(messages.state, messages.odometry, stamp_ns);
  update_twist(messages.state, messages.odometry, config);
  return messages;
}

Output output_for(
  const VehicleState & vehicle, std::uint64_t command_id, DriveMode mode, const Twist2d & target,
  TimestampNs computed_stamp_ns)
{
  Output output;
  output.command_id = command_id;
  output.observation_stamp_ns = vehicle.stamp_ns;
  output.computed_stamp_ns = computed_stamp_ns;
  output.valid_until_ns = *add_duration(computed_stamp_ns, .2);
  output.command = ChassisCommand{mode, target, std::nullopt};
  return output;
}

ApplicationEvidence application(std::uint64_t id, TimestampNs stamp_ns, TimestampNs delay_ns = 0)
{
  return {id, stamp_ns, stamp_ns + delay_ns};
}

void accepted(
  StateMessage & state, DriveMode mode, std::uint64_t id, const Twist2d & entry,
  const Config & config)
{
  state.request_id = id;
  state.requested_mode = static_cast<std::uint8_t>(mode);
  state.accepted_entry_velocity = wire(entry);
  state.accepted_steering = DriveModel(config).steering_for_entry(
    mode, {entry.vx, entry.vy, entry.wz}, state.steering_angles);
}

void test_parameter_contract()
{
  Config config;
  Parameters parameters;
  check(
    check_compatibility(config, parameters).compatible(), "matching chassis parameters rejected");
  config.samples_per_branch += 1;
  check(
    check_compatibility(config, parameters).compatible(),
    "planner-only configuration leaked into the chassis contract");
  parameters.wheel_radius_m = .11;
  const auto mismatch = check_compatibility(config, parameters);
  check(
    !mismatch.compatible() && mismatch.mismatches.front().name == "wheel_radius",
    "wheel geometry mismatch was not diagnosed");
  try {
    Adapter adapter(config, parameters);
    (void)adapter;
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("incompatible chassis parameters did not fail startup");
}

void test_state_mapping_and_admission()
{
  Config config;
  Adapter adapter(config, Parameters{});
  auto messages = startup(config);
  auto result = adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns);
  check(result.error == AdapterError::None && result.state, "startup state was rejected");
  check(
    std::abs(result.state->wheel_speeds[0] - .1) < 1e-12 &&
      std::abs(result.state->velocity.vx - .1) < 1e-12 && result.state->time_in_mode_s == 0,
    "raw wheel velocity was not converted to core SI units");

  stamp(messages.state, messages.odometry, 1100000000);
  result = adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns);
  check(
    result.error == AdapterError::None && result.state &&
      std::abs(result.state->time_in_mode_s - .1) < 1e-12,
    "confirmed mode age did not use source time");
  check(
    adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns).error ==
      AdapterError::OutOfOrder,
    "duplicate state was accepted");
  stamp(messages.state, messages.odometry, 1050000000);
  check(
    adapter.observe(messages.state, messages.odometry, 1100000000).error ==
      AdapterError::OutOfOrder,
    "out-of-order state was accepted");

  Adapter pairing(config, Parameters{});
  auto bad = startup(config);
  bad.odometry.stamp_ns += *duration_nanoseconds(.006);
  check(
    pairing.observe(bad.state, bad.odometry, bad.odometry.stamp_ns).error ==
      AdapterError::Unsynchronized,
    "unpaired state/odometry was accepted");
  bad = startup(config);
  check(
    pairing.observe(bad.state, bad.odometry, *add_duration(bad.state.stamp_ns, .151)).error ==
      AdapterError::Stale,
    "stale state was accepted");
  bad = startup(config);
  check(
    pairing.observe(bad.state, bad.odometry, bad.state.stamp_ns - 3000000).error ==
      AdapterError::Future,
    "future state was accepted");
  bad = startup(config);
  bad.odometry.child_frame_id = "wrong_base";
  check(
    pairing.observe(bad.state, bad.odometry, bad.state.stamp_ns).error ==
      AdapterError::InvalidFrame,
    "incorrect state frame was accepted");
  bad = startup(config);
  bad.state.velocity.linear_z = 1;
  check(
    pairing.observe(bad.state, bad.odometry, bad.state.stamp_ns).error ==
      AdapterError::InvalidState,
    "non-planar chassis velocity was accepted");
  bad = startup(config);
  bad.odometry.velocity.linear_x += .01;
  check(
    pairing.observe(bad.state, bad.odometry, bad.state.stamp_ns).error ==
      AdapterError::InconsistentState,
    "odometry/chassis velocity disagreement was accepted");
}

void test_publication_and_application_evidence()
{
  Config config;
  Adapter adapter(config, Parameters{});
  auto messages = startup(config);
  const auto observed = adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns);
  check(observed.state.has_value(), "command test requires an observation");
  const auto computed = *add_duration(observed.state->stamp_ns, .01);
  auto output = output_for(*observed.state, 1, DriveMode::DualAckermann, {.2, 0, 0}, computed);
  const auto published = *add_duration(computed, .001);
  const auto mapped = adapter.make_command(output, published);
  const auto applied = adapter.record_application(application(1, published, 10000000));
  check(
    mapped.error == AdapterError::None && mapped.message && applied.application &&
      mapped.message->request_id == 0 && mapped.message->mode == kDualAckermann &&
      applied.application->earliest_stamp_ns == published &&
      applied.application->latest_stamp_ns == published + 10000000 &&
      output.published_stamp_ns == published,
    "drive publication did not preserve lifetime/application evidence");
  check(
    adapter.make_command(output, published).error == AdapterError::StaleOutput,
    "published output was reused");

  stamp(messages.state, messages.odometry, *add_duration(messages.state.stamp_ns, .1));
  const auto next = adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns);
  auto mismatch = output_for(*next.state, 2, DriveMode::DualAckermann, {}, messages.state.stamp_ns);
  check(
    adapter.make_command(mismatch, messages.state.stamp_ns).error == AdapterError::None,
    "second output was not prepared");
  check(
    adapter.record_application(application(2, messages.state.stamp_ns, 20000001)).error ==
      AdapterError::InvalidApplication,
    "over-wide application interval was accepted");
  check(
    adapter.record_application(application(3, messages.state.stamp_ns)).error ==
      AdapterError::InvalidApplication,
    "mismatched command evidence was accepted");
  check(
    adapter.record_application(application(2, messages.state.stamp_ns)).error == AdapterError::None,
    "matching application evidence was rejected after an ID mismatch");

  stamp(messages.state, messages.odometry, *add_duration(messages.state.stamp_ns, .1));
  const auto newest = adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns);
  auto expired =
    output_for(*newest.state, 3, DriveMode::DualAckermann, {}, messages.state.stamp_ns);
  expired.valid_until_ns = *add_duration(messages.state.stamp_ns, .001);
  check(
    adapter.make_command(expired, *add_duration(messages.state.stamp_ns, .002)).error ==
      AdapterError::ExpiredOutput,
    "expired controller output was serialized");
  auto stale =
    output_for(*observed.state, 3, DriveMode::DualAckermann, {}, messages.state.stamp_ns);
  check(
    adapter.make_command(stale, messages.state.stamp_ns).error == AdapterError::StaleOutput,
    "output derived from an old observation was serialized");
}

void test_modes_receipts_retries_and_realign()
{
  Config config;
  Adapter adapter(config, Parameters{});
  auto messages = startup(config);
  auto observed = adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns);
  std::uint64_t command_id = 1;
  const auto issue_request = [&](DriveMode mode, std::uint64_t request_id, const Twist2d & entry) {
    auto output = output_for(*observed.state, command_id, mode, {}, observed.state->stamp_ns);
    output.command->mode_request = ModeRequest{request_id, mode, entry};
    auto mapped = adapter.make_command(output, observed.state->stamp_ns);
    ++command_id;
    check(
      mapped.error == AdapterError::None && mapped.message &&
        mapped.message->request_id == request_id &&
        mapped.message->entry_velocity.linear_x == entry.vx &&
        mapped.message->entry_velocity.linear_y == entry.vy &&
        mapped.message->entry_velocity.angular_z == entry.wz,
      "mode request mapping lost its immutable entry receipt");
  };
  const auto advance = [&](
                         DriveMode actual, DriveMode requested, std::uint64_t request_id,
                         const Twist2d & entry, bool ready) {
    stamp(messages.state, messages.odometry, *add_duration(messages.state.stamp_ns, .1));
    messages.state.actual_mode = static_cast<std::uint8_t>(actual);
    accepted(messages.state, requested, request_id, entry, config);
    messages.state.phase = ready ? kReadyPhase : kBrakingPhase;
    messages.state.confirmed = ready;
    messages.state.fault = kNoFault;
    messages.state.wheel_speeds_radps.fill(0);
    if (ready) messages.state.steering_angles = messages.state.accepted_steering;
    update_twist(messages.state, messages.odometry, config);
    observed = adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns);
    check(observed.error == AdapterError::None && observed.state, "valid mode state was rejected");
  };

  issue_request(DriveMode::Spin, 1, {0, 0, .6});
  advance(DriveMode::DualAckermann, DriveMode::Spin, 1, {0, 0, .6}, false);
  issue_request(DriveMode::Spin, 1, {0, 0, .6});
  advance(DriveMode::Spin, DriveMode::Spin, 1, {0, 0, .6}, true);
  auto drive =
    output_for(*observed.state, command_id, DriveMode::Spin, {0, 0, .4}, observed.state->stamp_ns);
  auto mapped = adapter.make_command(drive, observed.state->stamp_ns);
  ++command_id;
  check(
    mapped.message && mapped.message->request_id == 1 &&
      mapped.message->entry_velocity.angular_z == .6,
    "ordinary drive did not retain the accepted request receipt");

  advance(DriveMode::Spin, DriveMode::Spin, 1, {0, 0, .6}, true);
  issue_request(DriveMode::Crab, 2, {.2, .5, 0});
  advance(DriveMode::Spin, DriveMode::Crab, 2, {.2, .5, 0}, false);
  advance(DriveMode::Crab, DriveMode::Crab, 2, {.2, .5, 0}, true);
  issue_request(DriveMode::DualAckermann, 3, {.4, 0, .2});
  advance(DriveMode::Crab, DriveMode::DualAckermann, 3, {.4, 0, .2}, false);
  advance(DriveMode::DualAckermann, DriveMode::DualAckermann, 3, {.4, 0, .2}, true);

  // Automatic same-mode realignment retains the last accepted request and
  // requires zero while confirmation is absent.
  advance(DriveMode::DualAckermann, DriveMode::DualAckermann, 3, {.4, 0, .2}, false);
  auto hold =
    output_for(*observed.state, command_id, DriveMode::DualAckermann, {}, observed.state->stamp_ns);
  mapped = adapter.make_command(hold, observed.state->stamp_ns);
  ++command_id;
  check(
    mapped.message && mapped.message->request_id == 3 &&
      mapped.message->entry_velocity.linear_x == .4 &&
      mapped.message->entry_velocity.angular_z == .2,
    "same-mode realignment did not retain the accepted receipt");

  auto mutated = messages;
  stamp(mutated.state, mutated.odometry, *add_duration(messages.state.stamp_ns, .1));
  mutated.state.accepted_entry_velocity.angular_z = .1;
  update_twist(mutated.state, mutated.odometry, config);
  check(
    adapter.observe(mutated.state, mutated.odometry, mutated.state.stamp_ns).error ==
      AdapterError::InvalidTransition,
    "accepted receipt mutation was not rejected");
}

void test_fault_recovery_sequence()
{
  Config config;
  Adapter adapter(config, Parameters{});
  auto messages = startup(config);
  auto observed = adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns);
  stamp(messages.state, messages.odometry, *add_duration(messages.state.stamp_ns, .1));
  messages.state.phase = kFaultPhase;
  messages.state.confirmed = false;
  messages.state.fault = 3;
  messages.state.wheel_speeds_radps.fill(0);
  update_twist(messages.state, messages.odometry, config);
  observed = adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns);
  check(observed.state && observed.state->mode_fault, "chassis fault was not represented");

  const auto first = adapter.make_recovery_command(messages.state.stamp_ns);
  const auto retry = adapter.make_recovery_command(*add_duration(messages.state.stamp_ns, .01));
  check(
    first.message && retry.message && first.message->request_id == 1 &&
      retry.message->request_id == first.message->request_id && adapter.recovery_pending(),
    "recovery did not retry one immutable high-water request");

  stamp(messages.state, messages.odometry, *add_duration(messages.state.stamp_ns, .1));
  accepted(messages.state, DriveMode::DualAckermann, 1, {}, config);
  messages.state.phase = kBrakingPhase;
  messages.state.fault = kNoFault;
  update_twist(messages.state, messages.odometry, config);
  check(
    adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns).error ==
      AdapterError::None,
    "accepted recovery request was rejected");
  check(adapter.recovery_pending(), "recovery completed before measured confirmation");

  stamp(messages.state, messages.odometry, *add_duration(messages.state.stamp_ns, .1));
  messages.state.phase = kReadyPhase;
  messages.state.confirmed = true;
  check(
    adapter.observe(messages.state, messages.odometry, messages.state.stamp_ns).error ==
      AdapterError::None,
    "confirmed recovery state was rejected");
  check(!adapter.recovery_pending(), "confirmed recovery did not release the adapter gate");
}
}  // namespace

int main()
{
  try {
    test_parameter_contract();
    test_state_mapping_and_admission();
    test_publication_and_application_evidence();
    test_modes_receipts_retries_and_realign();
    test_fault_recovery_sequence();
    std::cout << "Current chassis interface regressions passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
