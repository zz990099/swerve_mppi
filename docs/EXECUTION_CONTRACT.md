# Standalone execution protocol (0.3)

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
Drive validation allows linear mode-projection error up to maximum commanded
wheel speed times steering_tolerance_rad, and angular error up to that bound
divided by module radius. This permits the small nonideal twist caused by measured
steering error while rejecting incompatible-mode drive.

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
| Drive | Require matching actual mode, aligned measured joints, bounded finite wheel speeds and consistent encoder-derived body twist within mode limits, allowing bounded steering tracking error. | Mask drive and continue the committed stop/alignment. |
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

Recovery is explicit: stop actuators independently, verify the actual mode and
stopped body/joints, drain old transport commands, call executor.reset(recovered)
with confirmed=true/fault=false, then controller.reset(). A moving, malformed or
unconfirmed recovery state is rejected. Reset does not reuse request IDs. No
fault is cleared merely because a late acknowledgement or target angle appears.

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
