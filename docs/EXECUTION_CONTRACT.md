# Standalone execution protocol (0.7)

This contract is independent of ROS, Nav2 and Gazebo. ModeExecutor is a reference
supervisor that can be used behind any transport or directly in core tests.
It issues actuator targets; it does not simulate actuators or certify tracking.

## Request and feedback

| Field | Meaning |
| --- | --- |
| Output::action | Drive, Brake, RequestMode, Hold or SafeStop. |
| Output::mode_request | Present only for RequestMode; complete immutable request. |
| ModeRequest::id | Nonzero, monotonically increasing per serialized executor session. |
| ModeRequest::mode | Explicit desired mode; never inferred from zero/nonzero twist. |
| ModeRequest::steering_targets | Frozen FL/FR/RL/RR mechanical angles derived from entry intent. |
| ModeFeedback::actual_mode | Last successfully completed mode, retained throughout a switch. |
| ModeFeedback::request_id | Executor's active/last accepted ID; completion also requires confirmed. |
| ModeFeedback::confirmed | Stable mode, with transition stop/alignment checks completed. |
| ModeFeedback::fault | Latched execution failure; requires deliberate recovery. |
| ModeFeedback::time_in_mode_s | Age of the actual mode; resets only when a new mode completes. |

VehicleState retains its public mode fields. Populate them from feedback as
shown below, alongside fresh measured pose, body velocity, steering and wheel
speeds. Request ID zero denotes startup state. ModeManager selects the next ID
above both its own high-water mark and VehicleState::mode_request_id. Both
manager and executor retain high-water marks across reset. A restarted process
must establish a fresh, drained transport session and obtain executor feedback
before enabling control; this is not a cross-process UUID protocol.

## Cycle

```cpp
#include <swerve_mppi/controller.hpp>
#include <swerve_mppi/executor.hpp>

swerve_mppi::Config config;
swerve_mppi::Controller controller(config);
swerve_mppi::ModeExecutor executor(config, initial_actual_mode);

// Once per config.dt_s, using fresh measurements and the last executor feedback:
const auto command = controller.compute(input);
const auto execution = executor.update(command, input.vehicle);
// Send execution.steering_targets and execution.wheel_speed_targets to actuators.
// After collecting the next measured pose/twist/encoders, map feedback:
input.vehicle.actual_mode = execution.feedback.actual_mode;
input.vehicle.mode_confirmed = execution.feedback.confirmed;
input.vehicle.mode_fault = execution.feedback.fault;
input.vehicle.mode_request_id = execution.feedback.request_id;
input.vehicle.time_in_mode_s = execution.feedback.time_in_mode_s;
// Advance stamp_s from the actual measurement clock, not wall-clock prediction.
```

This is a mapping sketch: initial_actual_mode, input and the actuator/measurement
functions belong to the caller. Initialize the executor with verified actual
mode; this does not measure or automatically discover the chassis configuration.
For a nondefault startup request ID, initialize it through reset(recovered) before
normal updates. Do not run two supervisors against the same actuators. A hardware
or simulator supervisor can implement the same protocol instead of ModeExecutor.

The first request is accepted only in Stable. A higher ID cannot replace an
active transition. The same ID and identical payload is an idempotent retry;
a changed payload, lower ID or malformed request latches a fault. Entry targets
must also match the requested mode: parallel wheel axes for Crab, tangential
axes for Spin, or a common Ackermann curvature respecting min_turn_radius_m.
A retry after completion returns a stopped acknowledgement without restarting mode age.
Replaying a completed request after the chassis has started moving is a fault.
In 0.4 a Drive's body_command must be finite and match forward kinematics of
wheel_speed_targets and steering_targets. These are the predicted next joint
positions/speeds, not a twist computed at the previous measured angles. Each
steering step is bounded by both drive_steering_limit_rad and
max_steer_rate_radps * dt_s relative to measurement. The actuator must track both
joint target arrays, including steering while driving; it must not gate all
steering changes on zero speed.

The mode-projection consistency allowance uses maximum commanded wheel speed
times drive_steering_limit_rad for linear error, and that bound divided by module
radius for angular error. This permits a bounded transient between steering
geometries; it is not a calibrated tire-slip model. Large steering jumps and
incompatible-mode body motion remain invalid. The model limits the complete
wheel/steering step against body acceleration, and the actuator layer continues
to own physical rate enforcement.

The transport must preserve command order and have one serialized command writer.

During braking, all drive targets are zero and steering retains measured angles.
Both body velocity and every measured wheel speed must be stopped. Once stopped,
the executor applies the frozen entry steering. Confirmation requires measured
angles within steering_tolerance_rad and at least alignment_min_s since entering
alignment. If wheels/body move again, it returns to braking and restarts the
alignment minimum, while retaining the original overall deadline. The reported
mode changes only at confirmation. Controller/ModeManager then verifies mode,
ID, stopped measurements and steering, and emits one stopped handover cycle.

## Actions, timing and cancellation

| Action | Stable execution | During a committed transition |
| --- | --- | --- |
| Drive | Require matching actual mode, finite twist/wheel targets, bounded moving-steering steps and consistency of the complete joint target step within mode limits. | Mask drive and continue the committed stop/alignment. |
| Brake | Zero drive targets, retain measured steering. | Continue the committed transition. |
| Hold | Zero drive; apply steering only when stopped, otherwise brake. | Continue the committed transition. |
| RequestMode | Start/retry the explicit request. | Retry the same request; reject replacement. |
| SafeStop | Zero drive and latch fault. | Cancel alignment, retain measured steering and latch fault. |

SafeStop is the explicit cancellation operation. An empty or invalid path also
causes Controller to emit SafeStop. Hold or a different DriveMode request cannot
cancel a committed transition or recover a fault.
Malformed measurements, nonincreasing timestamps and execution faults also latch
SafeStop. Executor fault outputs retain the last valid measured steering and
zero all drive targets; an actuator watchdog must enforce stopping even when the
calling process stops running. The core cannot detect silence without calls.

ModeManager's deadline starts when a switch is committed, including its braking.
ModeExecutor's deadline starts when it receives the first RequestMode, including
any local braking. Retries renew neither deadline. TransitionModel accounts for
braking, bounded alignment and confirmation_prediction_s; the last parameter is
a planning allowance for feedback/transport delay, not permission to synthesize
confirmation. It rejects predicted transitions beyond confirmation_timeout_s.
Feedback age, clock-reset handling and transport latency remain caller duties.
The reusable guarded entry point below enforces these checks when the caller
provides a current clock and periodic watchdog ticks.

Recovery is explicit: stop actuators independently, verify the actual mode and
stopped body/joints, drain old transport commands, call executor.reset(recovered)
with confirmed=true/fault=false, then controller.reset(). A moving, malformed or
unconfirmed recovery state is rejected. Reset does not reuse request IDs. No
fault is cleared merely because a late acknowledgement or target angle appears.

## Recoverable planning stops

NoFeasiblePlan is not itself an execution fault. Controller checks a complete
zero-control stopping trace through its hard TrajectoryValidator, including the
current footprint and every swept segment. If valid and stopped at its end, it
emits Brake/Hold with Waiting/Blocked, retains measured steering and permits fresh
replanning. Warm start and local alignment are cleared. If stopping is rejected
or cannot finish within the horizon, UnsafeStoppingTrajectory emits SafeStop.
Invalid inputs, model failures and handshake failures also remain SafeStop.
These predictive checks do not establish braking safety for uncalibrated actuators
or tire slip.

## Guarded timing and command envelopes

Use TimedExecutor instead of direct ModeExecutor for queued/transported commands.
Each CommandEnvelope has a nonzero session_id, a strictly increasing nonzero
sequence, a strictly increasing issued_at_s and the complete Output. RequestMode
retries retain their mode request ID but get new envelope sequences and issue
times; transport sequence and mode request ID are independent.

```cpp
#include <swerve_mppi/controller.hpp>
#include <swerve_mppi/timing.hpp>

swerve_mppi::Controller controller(config);
swerve_mppi::TimedExecutor executor(config, session_id, initial_actual_mode);
// One serialized call per model tick, with fresh measurements and clock_now_s:
const auto output = controller.compute(input);
swerve_mppi::CommandEnvelope envelope{session_id, ++sequence, clock_now_s, output};
const auto guarded = executor.update(envelope, input.vehicle, clock_now_s);
// Apply guarded.execution targets and map its ModeFeedback as in the cycle above.
// A watchdog tick without a new command uses update(std::nullopt, measured, now).
```

TimingLimits defaults are max_feedback_age_s=0.15, max_command_age_s=0.15 and
period_tolerance_ratio=0.25. Receiving tick intervals and measurement stamp intervals
must match dt_s within that relative tolerance. Ages at the limit are accepted.
Future/invalid timestamps, clock regressions, duplicate feedback, missed periods,
expired/replayed/missing commands and wrong sessions latch a distinct TimingError
and force SafeStop. The guard never repeats the last Drive. TimingGuard is also
available to reject feedback before planning; check_feedback then check_command
use the same current tick time.

Measurement, command and now timestamps must use one clock domain. The adapter
supplies the clock, ticks even when a command is missing, and maps feedback. A
paused simulation clock does not replace an independent actuator watchdog. The
core cannot enforce stopping if its process is not called.

Timing recovery requires independently verified stopped/confirmed feedback,
draining pending commands, executor.reset(recovered, new_session_id) with a strictly
larger session ID, and controller.reset(). Clock and transport sequence can restart;
mode request ID high-water marks remain intact. Old sessions are rejected even
with fresh timestamps. Across process restarts the caller must persist/coordinate
session identity and drain transport; this is not an automatic reconnect protocol.

## Regression boundaries

CTest's execution suite covers all six directed mode changes with an independent
rate-limited encoder fixture; frozen retries; stale acknowledgements; measured
steering/wheel gates; deadline latching; cancellation and ID replay after recovery;
mode persistence; valid entry geometry; request ID exhaustion; drive/encoder
consistency and steering tracking tolerance; backward clocks; curved Ackermann
and Spin progress; and controller to executor to measured-feedback lateral
progress. The fixture is deliberately small
and has no Gazebo, ROS, terrain, tire slip, communication transport or dynamics
calibration. Its results establish the core protocol behavior only.

## Same-mode steering commitment

A large steering change in the current mode produces Brake/Hold until it can
resume bounded Drive. Controller freezes the control intent across those cycles,
checks the continuation against fresh obstacles, and applies the original
confirmation_timeout_s bound. This does not issue a ModeRequest or reset mode
age. Reset or a new task path cancels obsolete local alignment. An explicit mode
request is preserved across replans; task cancellation still uses SafeStop/reset.

Tracking, corner and terminal policies share the same local commitment and timeout.
Changing policy cannot renew the deadline or discard an unfinished alignment.

Mode-entry intent is also retained for the first Drive after a confirmed switch,
so stochastic optimization cannot immediately undo the steering handshake.

## Navigation lifecycle in 0.5

Controller now owns ordered-path progress and measured goal completion. Navigation
status is separate from execution phase. Replanning clears local optimizer and
same-mode alignment state, while an active RequestMode retains its ID, complete
payload and original deadline. Completion requires stopped body/wheels and a
confirmed mode over a settling dwell; afterward Controller holds the task stopped.
See [NAVIGATION.md](NAVIGATION.md) for path identity, heading policy and diagnostics.
