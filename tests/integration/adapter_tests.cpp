#include <functional>
#include <iostream>
#include <limits>
#include <sstream>

#include "adapter_fixture.hpp"
#include "behavior_fixture.hpp"
using namespace swerve_mppi;
using namespace swerve_mppi::test;
namespace
{
void rejects(const std::function<void()> & action, const char * message)
{
  bool rejected = false;
  try {
    action();
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, message);
}
void test_resolved_exchange()
{
  Config c;
  const auto profile = write_config_profile(c);
  const auto restored = parse_resolved_config_profile(profile);
  require_execution_compatible(c, restored);
  std::vector<std::string> lines;
  std::istringstream input(profile);
  for (std::string line; std::getline(input, line);) {
    lines.push_back(line);
  }
  check(lines.size() == config_schema().size(), "resolved field coverage");
  for (std::size_t omitted = 0; omitted < lines.size(); ++omitted) {
    std::string partial;
    for (std::size_t i = 0; i < lines.size(); ++i) {
      if (i != omitted) {
        partial += lines[i] + '\n';
      }
    }
    rejects([&] { parse_resolved_config_profile(partial); }, "any omitted peer field must reject");
  }
  rejects([&] { parse_resolved_config_profile(""); }, "empty exchange cannot inherit defaults");
  rejects([&] { parse_resolved_config_profile(profile + lines[0]); }, "duplicate exchange field");
  rejects(
    [&] { parse_resolved_config_profile(profile + "future_field=1"); },
    "unknown peer schema field");
  check(
    parse_config_profile("random_seed=7").random_seed == 7,
    "partial local overrides remain supported");
  const AdapterContract local(profile, adapter_metadata());
  auto planning = c;
  planning.samples_per_branch += 1;
  require_adapter_compatible(
    local, AdapterContract(write_config_profile(planning), adapter_metadata()));
  auto incompatible = c;
  incompatible.max_linear_decel_mps2 *= .5;
  rejects(
    [&] {
      require_adapter_compatible(
        local, AdapterContract(write_config_profile(incompatible), adapter_metadata()));
    },
    "braking disagreement must reject before arming");
  for (int variant = 0; variant < 8; ++variant) {
    auto changed = adapter_metadata();
    if (variant == 0) {
      changed.world_frame = "map";
    }
    if (variant == 1) {
      changed.body_frame = "chassis";
    }
    if (variant == 2) {
      changed.clock.domain = "host_wall";
    }
    if (variant == 3) {
      ++changed.clock.epoch;
    }
    if (variant == 4) {
      changed.motion_policy = MotionEvidencePolicy::NominalEncoderOnly;
    }
    if (variant == 5) {
      changed.timing->max_feedback_age_s += .01;
    }
    if (variant == 6) {
      changed.timing->period_tolerance_ratio += .01;
    }
    if (variant == 7) {
      changed.watchdog_s += .1;
    }
    rejects(
      [&] { require_adapter_compatible(local, AdapterContract(profile, changed)); },
      "metadata disagreement");
  }
  for (int variant = 0; variant < 13; ++variant) {
    auto bad = adapter_metadata();
    if (variant == 0) {
      bad.schema_version = 0;
    }
    if (variant == 1) {
      bad.schema_version = 2;
    }
    if (variant == 2) {
      bad.world_frame.clear();
    }
    if (variant == 3) {
      bad.body_frame = bad.world_frame;
    }
    if (variant == 4) {
      bad.world_frame = std::string(129, 'a');
    }
    if (variant == 5) {
      bad.clock.domain = "space not allowed";
    }
    if (variant == 6) {
      bad.clock.epoch = 0;
    }
    if (variant == 7) {
      bad.motion_policy = MotionEvidencePolicy::Unspecified;
    }
    if (variant == 8) {
      bad.motion_policy = static_cast<MotionEvidencePolicy>(99);
    }
    if (variant == 9) {
      bad.timing.reset();
    }
    if (variant == 10) {
      bad.timing->max_command_age_s = 0;
    }
    if (variant == 11) {
      bad.timing->period_tolerance_ratio = 1;
    }
    if (variant == 12) {
      bad.watchdog_s = std::numeric_limits<double>::quiet_NaN();
    }
    rejects([&] { AdapterContract contract(profile, bad); }, "invalid metadata must fail closed");
  }
  c.compute_budget_ratio = 0;
  rejects(
    [&] { AdapterContract contract(write_config_profile(c), adapter_metadata()); },
    "live contract cannot disable compute budget");
}
void test_snapshot_metadata()
{
  const AdapterContract contract(write_config_profile(Config{}), adapter_metadata());
  auto p = stopped_packet();
  check(
    check_snapshot_contract(contract, p.metadata) == SnapshotContractError::None,
    "coherent frames and clocks");
  for (int source = 0; source < 5; ++source) {
    auto changed = p.metadata;
    auto * field = source == 0   ? &changed.pose
                   : source == 1 ? &changed.encoders
                   : source == 2 ? &changed.mode
                   : source == 3 ? &changed.context
                                 : &*changed.body_observation;
    field->frame = "wrong_frame";
    check(
      check_snapshot_contract(contract, changed) == SnapshotContractError::FrameMismatch,
      "source frame mismatch");
    field->frame = source == 0 || source == 3 ? "odom" : "base_link";
    ++field->clock.epoch;
    check(
      check_snapshot_contract(contract, changed) == SnapshotContractError::ClockMismatch,
      "source clock epoch mismatch");
    --field->clock.epoch;
    --field->stamp_ns;
    check(
      check_snapshot_contract(contract, changed) == SnapshotContractError::Unsynchronized,
      "source stamp mismatch");
    field->stamp_ns = -1;
    check(
      check_snapshot_contract(contract, changed) == SnapshotContractError::InvalidMetadata,
      "invalid source stamp");
  }
  p.metadata.application_clock.epoch = 2;
  check(
    check_snapshot_contract(contract, p.metadata) == SnapshotContractError::ClockMismatch,
    "application reset requires fresh contract");
  p = stopped_packet();
  ++p.metadata.application_stamp_ns;
  check(
    check_snapshot_contract(contract, p.metadata) == SnapshotContractError::StateNotCurrent,
    "recent source is not current");
  p = stopped_packet(1700000000000000000);
  ++p.metadata.pose.stamp_ns;
  check(
    check_snapshot_contract(contract, p.metadata) == SnapshotContractError::Unsynchronized,
    "original ns at large epochs");
  p = stopped_packet();
  p.metadata.body_observation.reset();
  check(
    check_snapshot_contract(contract, p.metadata) ==
      SnapshotContractError::MissingMotionObservation,
    "independent path cannot omit body metadata");
  const AdapterContract nominal(
    write_config_profile(Config{}), adapter_metadata(MotionEvidencePolicy::NominalEncoderOnly));
  check(
    check_snapshot_contract(nominal, p.metadata) == SnapshotContractError::None,
    "explicit nominal development path");
}
AdapterPacket arm(AdapterOwnerFixture & owner, std::int64_t start = 1000000000)
{
  for (int tick = 0; tick < 3; ++tick) {
    check(
      !owner.observe_stopped(stopped_packet(start + tick * 100000000), start + tick * 100000000),
      "insufficient stop dwell");
  }
  auto p = stopped_packet(start + 300000000);
  check(
    owner.observe_stopped(p, p.metadata.application_stamp_ns),
    "sustained current stopped evidence");
  check(
    owner.recover(p, owner.session() + 1, true, true, p.metadata.application_stamp_ns),
    "drained, independently stopped recovery");
  return p;
}
void test_ingress_revokes_drive()
{
  const AdapterContract contract(write_config_profile(Config{}), adapter_metadata());
  for (int variant = 0; variant < 15; ++variant) {
    AdapterOwnerFixture owner(contract, contract);
    auto p = arm(owner);
    auto first =
      owner.apply(p, adapter_command(p, owner.session(), 1), 10, p.metadata.application_stamp_ns);
    check(
      first && first->actuation && first->execution.action == Action::Drive,
      "healthy reference flow installs Drive");
    check(
      owner.sample(1.35, 10.05).has_value(), "previous Drive sample exists before ingress fault");
    auto bad = stopped_packet(1360000000);
    if (variant == 0) {
      bad.metadata.pose.frame = "map";
    }
    if (variant == 1) {
      bad.metadata.mode.clock.epoch = 2;
    }
    if (variant == 2) {
      --bad.pose.stamp_ns;
    }  // Payload binding, metadata still looks coherent.
    if (variant == 3) {
      --bad.joints.stamp_ns;
    }
    if (variant == 4) {
      bad.body.reset();
    }
    if (variant == 5) {
      bad.body->source = MotionSource::EncoderDerived;
    }
    if (variant == 6) {
      bad.body->linear_error_bound_mps = .001;
    }
    if (variant == 7) {
      bad.body->velocity.vx = .001;
    }
    if (variant == 8) {
      bad.body->velocity.vx = .1;
    }
    if (variant == 9) {
      bad.joints.names.back() = "wrong_joint";
    }
    if (variant == 10) {
      bad.joints.names.back() = bad.joints.names.front();
    }
    if (variant == 11) {
      bad.joints.velocities[0] = std::numeric_limits<double>::infinity();
    }
    if (variant == 12) {
      bad.context.reference_path.clear();
    }
    if (variant == 13) {
      bad.context.reference_path.resize(contract.config().max_path_points + 1);
    }
    if (variant == 14) {
      bad.context.obstacles.resize(contract.config().max_obstacles + 1);
    }
    check(
      !owner.apply(bad, adapter_command(bad, owner.session(), 2, true), 10.06, 1360000000),
      "fault blocks new mode admission");
    check(
      owner.fault() && owner.stop_requests() > 0 && !owner.sample(1.36, 10.06),
      "ingress rejection immediately revokes the earlier still-live profile");
    check(
      !owner.apply(
        stopped_packet(1400000000), adapter_command(bad, owner.session(), 3), 10.1, 1400000000),
      "fresh healthy data cannot implicitly clear the owner fault");
  }
}
void test_current_callback_time()
{
  const AdapterContract contract(write_config_profile(Config{}), adapter_metadata());
  AdapterOwnerFixture owner(contract, contract);
  const auto p = arm(owner);
  check(
    !owner.decode(p, p.metadata.application_stamp_ns + 1),
    "self-consistent old metadata cannot define the owner's current clock");
  check(
    !owner.apply(
      p, adapter_command(p, owner.session(), 1), 10, p.metadata.application_stamp_ns + 1) &&
      owner.fault(),
    "stale whole packet cancels before nominal admission");
  AdapterOwnerFixture recovering(contract, contract);
  for (int tick = 0; tick < 4; ++tick) {
    recovering.observe_stopped(
      stopped_packet(1000000000LL + tick * 100000000LL), 1000000000LL + tick * 100000000LL);
  }
  check(
    !recovering.recover(p, 2, true, true, 1300000001),
    "old stopped evidence cannot arm at a later callback instant");
  for (int tick = 0; tick < 4; ++tick) {
    recovering.observe_stopped(
      stopped_packet(2000000000LL + tick * 100000000LL), 2000000000LL + tick * 100000000LL);
  }
  check(
    !recovering.recover(stopped_packet(2300000000), 2, true, false, 2300000000),
    "endpoint stopped acknowledgement is required");
}
void test_command_lifetime_and_fallback()
{
  const AdapterContract contract(write_config_profile(Config{}), adapter_metadata());
  for (int variant = 0; variant < 7; ++variant) {
    AdapterOwnerFixture owner(contract, contract);
    auto p = arm(owner);
    auto command = adapter_command(p, owner.session(), 1);
    if (variant == 0) {
      --command.session_id;
    }
    if (variant == 1) {
      command.sequence = 0;
    }
    if (variant == 2) {
      command.source_stamp_s = 0;
    }
    if (variant == 3) {
      command.valid_until_s = 1.2;
      command.execute_at_s = 1.2;
      command.issued_at_s = 1.2;
      command.source_stamp_s = 1.2;
    }
    if (variant == 4) {
      command.execute_at_s = 1.4;
      command.valid_until_s = 1.425;
    }
    if (variant == 5) {
      command.command.command.reset();
    }
    const auto result = owner.apply(
      p, variant == 6 ? std::nullopt : std::optional<CommandEnvelope>(command), 10,
      p.metadata.application_stamp_ns);
    check(
      result && !result->actuation && owner.fault() && !owner.sample(1.3, 10),
      "invalid command lifetime cannot retain Drive");
  }
  AdapterOwnerFixture owner(contract, contract);
  auto p = arm(owner);
  auto obsolete = adapter_command(p, owner.session(), 1, true);
  obsolete.source_task->path_id = 99;
  const auto fallback = owner.apply(p, obsolete, 10, p.metadata.application_stamp_ns);
  check(
    fallback && fallback->actuation &&
      fallback->safety_error == ExecutionSafetyError::TaskMismatch &&
      fallback->execution.action != Action::Drive && fallback->execution.feedback.request_id == 0 &&
      !owner.fault(),
    "obsolete request can only install a separately checked non-driving fallback");
  p = stopped_packet(1400000000);
  check(
    owner.apply(p, adapter_command(p, owner.session(), 2), 10.1, p.metadata.application_stamp_ns)
      ->actuation.has_value(),
    "fresh current task recovers healthy fallback");
  p = stopped_packet(1500000000);
  auto replay = adapter_command(p, owner.session(), 2);
  check(
    !owner.apply(p, replay, 10.2, p.metadata.application_stamp_ns)->actuation && owner.fault(),
    "sequence replay latches");
}
void test_independent_stop_threshold()
{
  Config c;
  c.stopped_wheel_speed_mps = .02;
  const AdapterContract contract(write_config_profile(c), adapter_metadata());
  AdapterOwnerFixture owner(contract, contract);
  const auto wheels = Kinematics(c).inverse({0, 0, c.stopped_angular_radps - 2e-10}, {});
  const auto encoded = Kinematics(c).forward(wheels.speeds, wheels.angles);
  for (int tick = 0; tick < 4; ++tick) {
    auto p = stopped_packet(1000000000LL + tick * 100000000LL);
    p.mode.actual_mode = DriveMode::Spin;
    for (std::size_t i = 0; i < 4; ++i) {
      p.joints.velocities[2 * i] = wheels.speeds[i] / c.wheel_radius_m;
      p.joints.positions[2 * i + 1] = wheels.angles[i];
    }
    p.body->velocity = encoded;
    p.body->velocity.wz += 5e-10;
    const auto decoded = owner.decode(p, p.metadata.application_stamp_ns);
    check(
      decoded && is_stopped(decoded->vehicle, c), "numerically agreeing encoded stop threshold");
    check(
      !owner.observe_stopped(p, p.metadata.application_stamp_ns),
      "independent body threshold cannot be replaced by encoder stop even within nominal epsilon");
  }
}
void test_explicit_profile_cancel()
{
  Config c;
  const AdapterContract contract(write_config_profile(c), adapter_metadata());
  AdapterOwnerFixture decoder(contract, contract);
  auto p = stopped_packet();
  auto input = *decoder.decode(p, 1000000000);
  TimedExecutor executor(c, 1);
  ProfileRunner runner(c);
  const auto initial = executor.update(adapter_command(p, 1, 1), input, 1);
  check(
    runner.install(initial, 1, 10) && runner.sample(1.05, 10.05), "cancel fixture live profile");
  runner.cancel();
  runner.cancel();
  check(
    runner.fault() && !runner.sample(1.06, 10.06),
    "idempotent cancellation revokes within live interval");
  p = stopped_packet(1100000000);
  input = *decoder.decode(p, 1100000000);
  const auto later = executor.update(adapter_command(p, 1, 2), input, 1.1);
  check(
    later.actuation && !runner.install(later, 1.1, 10.1),
    "healthy installation cannot clear cancel latch");
  executor.reset(input.vehicle, 2);
  runner.reset(input.vehicle);
  const auto recovered = executor.update(adapter_command(p, 2, 1), input, 1.1);
  check(
    runner.install(recovered, 1.1, 10.1) && runner.sample(1.15, 10.15),
    "explicit stopped reset and fresh session permit a new profile");
}
void test_watchdogs_and_recovery()
{
  const AdapterContract contract(write_config_profile(Config{}), adapter_metadata());
  for (int variant = 0; variant < 4; ++variant) {
    AdapterOwnerFixture owner(contract, contract);
    const auto p = arm(owner);
    check(
      owner.apply(p, adapter_command(p, owner.session(), 1), 10, p.metadata.application_stamp_ns)
        ->actuation.has_value(),
      "watchdog fixture Drive");
    const double now = variant == 0 ? 1.3 : variant == 1 ? 1.40001 : variant == 2 ? 1.35 : 1.29;
    const double wall = variant == 0 ? 10.50001 : variant == 1 ? 10.1 : variant == 2 ? 9.99 : 10.05;
    check(
      !owner.sample(now, wall) && owner.fault(),
      "wall pause, interval expiry or wall rollback cancels");
  }
  AdapterOwnerFixture owner(contract, contract);
  check(!owner.sample(1, 10), "startup cannot sample a profile");
  auto p = stopped_packet();
  check(
    !owner.observe_stopped(p, p.metadata.application_stamp_ns), "one stopped packet cannot arm");
  check(
    !owner.observe_stopped(p, p.metadata.application_stamp_ns),
    "duplicate observation breaks continuous dwell");
  for (int i = 1; i <= 4; ++i) {
    owner.observe_stopped(
      stopped_packet(1000000000LL + i * 100000000LL), 1000000000LL + i * 100000000LL);
  }
  p = stopped_packet(1400000000);
  check(
    !owner.recover(p, 2, false, true, p.metadata.application_stamp_ns),
    "transport drain acknowledgement is required");
  p = arm(owner, 2000000000);
  const auto old_session = owner.session();
  owner.trip();
  // Motion during the recovery window and skipped ticks reset dwell.
  auto moving = stopped_packet(2400000000);
  moving.body->velocity.vx = .04;
  check(
    !owner.observe_stopped(moving, 2400000000),
    "body coasting cannot recover despite stopped encoders");
  check(!owner.observe_stopped(stopped_packet(2500000000), 2500000000), "new dwell after motion");
  check(
    !owner.observe_stopped(stopped_packet(2800000000), 2800000000),
    "gap cannot count as observed stop time");
  for (int i = 0; i < 4; ++i) {
    owner.observe_stopped(
      stopped_packet(3000000000LL + i * 100000000LL), 3000000000LL + i * 100000000LL);
  }
  p = stopped_packet(3300000000);
  check(
    !owner.recover(p, old_session, true, true, p.metadata.application_stamp_ns),
    "recovery requires a strictly newer session");
  p = arm(owner, 4000000000);
  auto delayed = adapter_command(p, old_session, 1);
  check(
    owner.apply(p, delayed, 20, p.metadata.application_stamp_ns)->timing_error ==
        TimingError::SessionMismatch &&
      owner.fault(),
    "drained old-session packet still cannot revive Drive");
  p = arm(owner, 5000000000);
  const auto resumed =
    owner.apply(p, adapter_command(p, owner.session(), 1), 30, p.metadata.application_stamp_ns);
  check(
    resumed && resumed->actuation && owner.sample(5.35, 30.05),
    "fresh session resumes after explicit recovery");
}
}  // namespace
int main()
{
  try {
    test_resolved_exchange();
    test_snapshot_metadata();
    test_ingress_revokes_drive();
    test_current_callback_time();
    test_command_lifetime_and_fallback();
    test_independent_stop_threshold();
    test_explicit_profile_cancel();
    test_watchdogs_and_recovery();
    std::cout << "adapter startup, ingress, lifetime, revocation and recovery checks passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
