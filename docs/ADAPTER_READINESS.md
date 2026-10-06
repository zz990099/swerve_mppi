# Adapter contract and ownership (0.21.4)

This is the transport-free contract for later adapter migration. The core has no
ROS node, TF buffer, DDS serializer or physical stop channel here. The companion
repository is unchanged. Existing integrations do not gain these checks simply by
upgrading the library; the owner must exchange, bind and enforce the contract.

The existing navigation/planning/model/safety/execution separation remains.
`integration/adapter_contract.hpp` joins their boundary assumptions without
becoming another optimizer, mode supervisor or actuator owner. Nav2-style lifecycle
and optimizer ownership remain useful; the explicit swerve mode handshake,
execution-time complete-stop checks and independent actuator watchdog must survive
any later wrapper/plugin implementation.

## Startup exchange

Each participant resolves its complete local Config and sends
`write_config_profile(config)` plus explicit AdapterMetadata. Decode a received
profile with `parse_resolved_config_profile` or the AdapterContract constructor:
all registered fields must occur exactly once. Missing parameters cannot inherit
local defaults. Local override files still use `parse_config_profile`; they are not
peer exchange payloads. Unknown, duplicate, malformed, invalid and oversize profiles
are rejected. Both peers must support this field set and adapter schema version 1.

```cpp
#include <swerve_mppi/integration/adapter_contract.hpp>

swerve_mppi::AdapterMetadata metadata{
  1, "odom", "base_link", {"simulation_clock", 1},
  swerve_mppi::MotionEvidencePolicy::IndependentNominal,
  swerve_mppi::TimingLimits{}, .5};
const swerve_mppi::AdapterContract local(
  swerve_mppi::write_config_profile(resolved_local_config), metadata);
const swerve_mppi::AdapterContract peer(received_resolved_profile, received_metadata);
swerve_mppi::require_adapter_compatible(local, peer);
// Agreement permits proceeding to stopped recovery/arming checks only.
```

AdapterContract is immutable, validates live compute budget, timing and watchdog
limits at construction, and owns all values. Comparisons require exact agreement
on execution-scope configuration, frames, clock identity, motion evidence policy,
TimingLimits and watchdog. Planning-only parameters may differ; both live budgets
must remain positive. A digest is useful evidence but cannot replace comparisons.
Identifiers are exact, case-sensitive strings of 1..128 ASCII letters/digits or
`_ - . / :`; no alias, slash normalization or TF equivalence is inferred. World
and body frame names must differ. Schema, policy, timing, epoch and watchdog cannot
be omitted from wire decoding; empty/default AdapterMetadata is invalid.

ClockIdentity names the application clock domain and a nonzero reset incarnation.
It is distinct from the monotonically increasing execution session and endpoint
session. Domain strings do not authenticate the physical clock. All source clock
identities and session ownership must be bound by the transport. After clock reset
or a contract/configuration change, disarm, stop, drain, perform a fresh exchange,
recreate affected core consumers, and use renewed transport/execution sessions.
Do not reuse the old contract or automatically arm upon equality. Persist or
coordinate session high-water marks across process restart; schema version 1 is
not a UUID/authentication protocol or a runtime ABI negotiation mechanism.

Select motion policy explicitly:

- NominalEncoderOnly supports known nominal development fixtures. It cannot establish
  independent localization/slip or physical stopping acceptance.
- IndependentNominal requires an independently sourced body observation with exact
  timestamp coherence, zero declared uncertainty and nominal numerical agreement.
  Nonzero uncertainty or even a small measured residual remains inadmissible.
  Noisy hardware is not ready for this nominal-only independent policy without a
  reviewed uncertainty-aware model. See MODEL_FEEDBACK.md.

If an independent observation is available and disagrees, do not ignore it by
selecting the encoder development policy. Contract changes require disarming and
review; policy selection cannot reinterpret evidence.

## Snapshot binding and frames

| Payload | Frame and units | Source stamp/ownership |
| --- | --- | --- |
| Pose | Contract world frame; m, m, rad | Original pose observation, never a predicted pose relabeled as measured |
| Encoders | Contract body/module convention; FL, FR, RL, RR | One complete JointObservation; steering rad, wheel rad/s converted with resolved radius |
| Independent motion | Contract body frame; vx/vy m/s, wz rad/s | IndependentBody source with justified deterministic error bounds |
| Mode status | Contract body/mechanism convention | Current executor status snapshot; actual mode, confirmed/fault, request ID/age and full immutable receipt |
| Path and static circles | Contract world frame; m and rad | Current task/context snapshot; path identity/heading policy and complete constraints preserved |
| Application instant | Contract application clock | Independently read by the owner at the callback boundary |

SnapshotMetadata carries StampedFrame records for pose, encoders, mode, context and
optional independent motion. Context stamp means the current assembled context;
it does not fabricate a new sensor timestamp for static geometry. Mode receipt
may persist through completion; the mode status snapshot must still be current.
Keep source observation times and accepted-request origin separate. An asynchronous
status/pose/joint stream needs a reviewed time-alignment pipeline, not an arbitrary
freshness tolerance or timestamp rewrite. Dynamic obstacle prediction is absent.

Call `check_snapshot_contract` before converting to seconds. It checks exact
source frames, domains/epochs and original integer stamp equality; all sources
must describe the declared application instant. Missing independent motion metadata
is rejected by IndependentNominal. The check is intentionally stricter than merely
being younger than TimingLimits. It does not perform TF transformations, authenticate
sources, assess physical validity, inspect payloads or enforce callback scheduling.

The owner must additionally compare the declared application stamp to its actual
clock reading, bind every StampedFrame to the decoded value, and preserve source
integer timestamps in FeedbackAdapter. A self-consistent stale packet does not
become current by defining its own application time. Reject oversized joint/task/obstacle packets before allocating or copying them.
Body observation frame/time
metadata must be associated with the actual MotionObservation; its source label
and uncertainty bounds are checked by MotionObserver. Noisy bounds remain
inadmissible to nominal control. Frame mismatch must withhold normal actuation;
any transformation must occur explicitly before creating a new coherent snapshot,
with its provenance retained. No stale stamp is refreshed by conversion.

FeedbackAdapter validates required named joints, same source timestamps and units,
but derives nominal velocity from encoders. For independent policy, assess the
separate body observation before nominal Controller/TimedExecutor calls. Copy mode
status, including accepted_mode_request, from the sole executor. Do not reconstruct
accepted entry geometry from current steering or infer a mode from Twist.

## Command lifetime and serialization

The future wire command must bind the complete Output to contract clock identity,
execution session, monotonically increasing sequence, original source stamp,
issue/application/start-expiry times and immutable CommandTask::capture of the
originating planning input. Transport decoding must reject missing fields before
constructing CommandEnvelope; never fill gaps with a current time or current task.
Command clock domain/epoch must match the active contract before calling the core.

Maintain `source <= issued <= execute <= valid_until`, preserve original source time
through every queue/retry, and keep the consumer's current clock independent from
the envelope. valid_until is the last admissible installation/start instant; it is
not permission to extend or repeat an installed profile. TimedExecutor recompiles
and checks the actual interval and complete stopping tail at the current state and
constraints. ProfileRunner samples that checked interval only. No checked profile,
no sample or an expired sample cannot retain previous Drive.

| Event | Owner behavior |
| --- | --- |
| Ordinary valid zero target | Nominal Brake/Hold; retain actual mode |
| Missing/absent command, invalid lifetime, wrong session, replay, bad snapshot/motion or endpoint fault | Revoke profile, latch and request independent stop |
| TaskMismatch/CommandRejected with separately checked non-driving fallback | May install that fallback; report execution diagnostics; never claim obsolete Drive/completion |
| Rejected new mode request | Do not commit request ID, entry geometry or mode |
| Previously committed transition during healthy fallback | Keep original ID/entry geometry/deadline; a replan does not cancel or renew it |
| Profile replacement | Install only at checked application boundary, serialize with sampling, discard older samples |

No new protocol supervisor is introduced. ModeExecutor/TimedExecutor remain the
sole owners of accepted entry geometry and confirmation. The transport must not
run a second Twist-to-mode inferencer or reuse a cached endpoint array.

## Stop, watchdog and recovery

`ProfileRunner::cancel()` explicitly clears the installed profile and latches its
fault without requiring a malformed TimedExecutionResult or another model tick.
It is idempotent and serial, has no physical braking effect, and cannot recover.
Call it on external ingress/independent-motion/endpoint faults, cancellation and
shutdown before publishing any more samples. Reset requires separately verified
stopped/confirmed feedback. Clearing the runner alone cannot reset execution or
transport sessions; resetting a healthy profile cannot revive an old queued plan.

| Responsibility | Required owner |
| --- | --- |
| Planning cadence and compute deadline | Serialized planning callback; no blocking actuator sampling |
| Feedback/envelope age, replay/session and task checks | Serialized TimedExecutor tick, including missing-command ticks |
| High-rate installed interval and wall-time watchdog | ProfileRunner install/sample/cancel/reset, serialized or protected by a short lock |
| Total process/callback silence | Independent actuator endpoint watchdog; cannot depend on MPPI callbacks |
| Physical emergency stop and minimum braking capability | Endpoint/plant owner with identified dynamics; no nominal collision certificate implied |
| Fault publication and recovery authorization | One adapter owner with endpoint acknowledgement and drained transport |

A paused application clock still allows steady-clock watchdog expiry. Both sampling
clocks must be finite/nonnegative/nondecreasing. Simulated time is not a substitute
for monotonic wall time. A stale packet, missed tick, deadline, motion disagreement
or clock rollback may revoke normal actuation; the independent endpoint still has
to stop if the adapter itself dies. Reject samples retained by another thread or
publisher after cancellation; a returned JointTargets value is not a revocable
capability. Serialize the final check and publication with fault/profile ownership.

Startup and recovery remain unarmed until a supervisor has:

1. Exchanged compatible complete contracts and verified exclusive actuator ownership.
2. Requested stop, revoked profiles and drained old command/sample queues.
3. Obtained fresh coherent pose/joints/mode and independent body evidence over a
   continuous stopping dwell. Moving or repeated/stale samples and skipped intervals
   break dwell; encoder-only stopped feedback cannot establish independent stop.
4. Verified the actual mode/plant and received matching endpoint stopped acknowledgement.
   Bind acknowledgement to the current endpoint/session and recovery attempt; a
   boolean or an old status packet does not prove physical stop.
5. Renewed the execution session, reset Controller, TimedExecutor and ProfileRunner,
   and rearmed the endpoint with a coordinated fresh session. Preserve request-ID
   high-water marks. Recovery acknowledgement alone grants no Drive permission.
6. Received a new valid current-task command and checked interval before streaming.

The reference fixture uses `goal_settle_time_s` as its minimum continuously observed
stop dwell; it does not replace mode alignment_min_s, minimum_mode_dwell_s or their
physical verification. A real owner may require a stricter independently identified
recovery dwell. It must enforce sampling cadence and truthful mode feedback rather
than copying the fixture's acknowledgements.

## Offline reference and readiness boundary

`tests/fixtures/adapter_fixture.hpp` is a serialized transport-free owner example.
It combines peer comparison, integer ingress binding, independent observation,
TimedExecutor and ProfileRunner; external faults revoke active Drive before another
model tick. Tests supply the actual callback clock independently, abstract endpoint
acknowledgement/queue-drain evidence and known stopped snapshots. It deliberately
contains no ROS, threads, servo, endpoint watchdog or physical stop proof. It is a
contract fixture, not a production adapter to deploy.

Build and run with the normal CMake entry point:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build -R 'adapter_regressions|configuration_regressions|installed_consumer' --output-on-failure
```

See ADAPTER_ACCEPTANCE.md for covered core requirements, evidence limitations and
remaining migration/physical acceptance work. Preparation stages 1..4 are complete.
Stop here until further integration is explicitly requested.
