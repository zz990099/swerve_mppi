# Chassis command contract (0.20)

The public planning result is `Output`, containing `PlanningDiagnostics` and an
optional `ChassisCommand`. It has no `action`, steering angles or wheel speeds.
The controller still observes joints and predicts their dynamics; removing joint
output does not remove actuator constraints or measured mode confirmation.

| Command field | Meaning |
| --- | --- |
| `mode` | DualAckermann, Crab or Spin. Ordinary velocity uses the actual confirmed mode. |
| `target_velocity` | Nominal body-frame `vx`, `vy` in m/s and `wz` in rad/s. |
| `mode_request` | Optional request with nonzero `id`, explicit `mode`, frozen `entry_velocity`. |

DualAckermann permits longitudinal velocity and bounded curvature/yaw; Crab permits
translation with zero yaw rate; Spin permits yaw rate with zero translation.
Targets must satisfy DriveModel's mode and speed limits. They are not wheel-encoder
FK endpoints: acceleration/steering limits can make the next measured velocity
smaller than the target. No mode is inferred from a velocity's direction.

## Actionless semantics

| Public command | Execution meaning |
| --- | --- |
| Present, nonzero target, no request | Compile the target in the current mode; align first if necessary, drive only when feedback permits. |
| Present, zero target, no request | Normal controlled braking/holding; retain actual mode and measured steering unless an already committed transition continues. |
| Present, zero target, explicit request | Brake, align using frozen entry intent, acknowledge ID/mode from measurements; no drive permission. |
| Absent command | Cancel and latch execution fault; use the independent stop path, never reuse an old target. |
| Missing envelope on a watchdog tick | Transport loss; stop and latch timing fault. |

An ordinary zero command cannot cancel an active mode transition. Explicit absence
cancels it. A fault cannot be cleared by a later zero or nonzero target. Recovery
requires an independently verified stopped/confirmed state, drained queues, renewed
transport session and controller/executor/profile reset. Request ID high-water
marks survive reset. Diagnostics describe planning but do not authorize execution.

```cpp
using namespace swerve_mppi;
Output drive;
drive.command = ChassisCommand{DriveMode::DualAckermann, {.3, 0, 0}, std::nullopt};

Output normal_stop;
normal_stop.command = ChassisCommand{actual_mode, {}, std::nullopt};

Output switch_to_crab;
switch_to_crab.command = ChassisCommand{
    DriveMode::Crab, {}, ModeRequest{next_request_id, DriveMode::Crab, {0, .3, 0}}};

Output cancel; // command is absent; it must not be treated as a normal zero Twist.
```

The request's `entry_velocity` defines entry steering geometry, not a drive command.
Command mode and request mode must agree; target_velocity must be exactly zero.
The complete request is immutable across retries, including entry speed magnitude.
The execution adapter freezes the derived mechanical geometry at first accepted
application. Same-ID mutation, lower/replayed IDs and transition replacement fault.
Retries cannot renew deadlines. After confirmation, the planner observes matching
actual mode, confirmed flag, request ID and stopped/aligned feedback before sending
its stopped handover and subsequent ordinary body target.

In 0.20, execution returns `ModeFeedback::accepted_mode_request`, the exact
immutable `JointModeRequest` it first accepted (ID, mode, mechanical steering
positions and original body entry velocity). Copy it to
`VehicleState::accepted_mode_request` with the other executor status fields.
Before acceptance, prediction selects geometry at the latest measured snapshot.
After acceptance, the planner validates the receipt against its frozen body intent
and original rolling lines modulo pi, then uses the receipt's exact positions in
retry safety checks. It never infers the commitment from moving measured angles.

The receipt remains present through completion and ordinary driving, until a
new request is accepted or deliberate executor recovery clears it. A pending
matching accepted ID without the receipt faults; once bound, missing or mutated
receipt data faults as well. A different accepted signed representation is legal,
but handover still requires exact measured alignment to the accepted positions,
matching confirmed mode/ID and stopped motion. Handover retains measured steering.

Nonzero entry steering is computed from normalized direction/curvature, regardless
of speed magnitude, including subnormal finite values. Exactly zero entry intent
uses the canonical geometry for the selected mode. The original entry velocity
is echoed without normalization and remains immutable across retries.

## Execution ownership

Controller's private planner checks predictions and complete stopping continuations
using DriveModel/ActuationModel. `ChassisExecutor` uses the same model to compile a
body command at the actual snapshot and apply mode supervision; it is an offline
reference entry point without timing, source-task or independent collision guards.
Integration must use `TimedExecutor`: it compiles the target, previews supervision
transactionally, revalidates the resulting actual joint interval and complete stop
against current obstacles/constraints, and commits only accepted execution.

Compiler request caches and protocol state are committed together. A task/collision
rejection cannot consume an unexecuted request ID or entry intent. An already active
request remains immutable through a separately checked non-driving fallback.
`TimedExecutionResult::actuation`, then ProfileRunner, provides the checked high-rate
joint reference to the actuator endpoint. Never invert the target with an unrelated
kinematic implementation and bypass those trajectory/rate checks.

`Action`, `JointCommand`, `JointModeRequest` and `ExecutionResult` remain lower-layer
model/supervisor APIs. Their internal Drive/Brake/Hold/RequestMode/SafeStop states are
necessary for the different interpolation and fault contracts. They are not fields
of the MPPI planning output or requirements for a ROS chassis command message.

## Migration and verification

The current package is 0.20; use `find_package(swerve_mppi 0.20 CONFIG REQUIRED)`
and module-qualified headers from [DEVELOPMENT.md](DEVELOPMENT.md).
The added feedback field changes the public aggregate layout: rebuild all consumers
and preserve the complete accepted request in ROS status conversion.
`FeedbackAdapter` copies it automatically; manual mappings must do so explicitly.
For the 0.18 command migration: Replace old
`Output.action/body_command/steering_targets/wheel_speed_targets/mode_request`
access with `Output.command`, target_velocity and its optional mode_request.
Replace direct `ModeExecutor.update(Output, state)` with ChassisExecutor for raw
tests, or TimedExecutor and its complete envelope for integration. Do not substitute
old body_command FK for target_velocity. Source/task/time metadata is still required.

`chassis_regressions` checks the public field boundary, nominal target versus FK,
shared prediction/compilation in all modes and signed reverse, same-mode alignment, all six directed mode handshakes,
zero versus absence, fault recovery, immutable retries/timeouts/ID preservation,
malformed targets and feedback, task rejection transactionality, transport loss and
fresh obstacle checks and task-independent cancellation. All 14 scenarios × five seeds × raw/timed/profile paths now
use public body commands. Joint/model regressions additionally exercise private
prediction and low-level admission to retain detailed malformed-joint coverage.
The installed consumer uses public headers only.

This is a portable library boundary, not a ROS message package or a completed Gazebo
MPPI planning node. The ROS adapter must preserve command presence, mode requests,
acknowledgements and complete envelope metadata; a bare Twist is insufficient.
Physical profile tracking, slip/latency/braking calibration and continuous-time
measurement alignment remain simulation acceptance work.
