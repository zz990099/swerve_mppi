# Chassis command contract (0.18)

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

Rebuild against 0.18 (`find_package(swerve_mppi 0.18 CONFIG REQUIRED)`). Replace old
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
