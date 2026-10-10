# Current chassis offline adapter

`current_chassis::Adapter` is the ROS-independent boundary between the MPPI core
and the current Python chassis command/state schema. Its DTOs mirror only the
fields required from `ChassisCommand`, `ChassisState` and encoder odometry. The
adapter does not import ROS, launch Gazebo, publish topics or operate joints.

## State ingress

`observe(state, odometry, planning_stamp_ns)` pairs chassis state with odometry,
checks source time, frames, mode/fault/phase consistency and monotonic request
identity, then returns a `VehicleState`. Raw wheel `rad/s` is converted to rolling
`m/s`; body velocity is recomputed from the same wheels and steering angles and
must agree with both wire twists. Duplicate/out-of-order samples, stale/future
data, mutated accepted receipts and invalid transitions are rejected.

The current wire state has no mode-age field. The adapter conservatively starts
mode age at zero on the first confirmed sample and after every loss of confirmation,
mode change or request change. Only a continuous sequence of confirmed samples for
the same actual mode and request accumulates `time_in_mode_s`.

## Command egress

`make_command(output, publication_stamp)` accepts exactly one current `Output` for
exactly one latest observation. It records publication once, rejects expired or
reused results and maps only body velocity plus the current mode request. Ordinary
drive and zero hold retain the chassis-owned request ID and accepted entry velocity.
Retries retain the planner request ID and entry; a pending accepted receipt cannot
be mutated.

After transport publication, `record_application(evidence)` validates an exact or
bounded application interval against the recorded command ID, publication, expiry
and configured uncertainty, then returns `CommandApplication` for the next input.
Calling it before evidence exists is not required.

`ApplicationEvidence` is deliberately explicit. The current chassis message does
not echo MPPI `command_id`, so the adapter cannot infer an exact application time
from a later `ChassisState`. A transport may report an exact callback/application
timestamp, or a bounded interval only when its scheduling contract justifies that
bound. The result supplies the corresponding `CommandApplication` for the next
controller input. If the next observation precedes the interval end, do not call
the controller yet. Unknown application remains missing evidence; it must never be
relabeled as publication time.

## Recovery

A chassis fault cannot be cleared by a normal controller output. Call
`make_recovery_command` after a fresh fault observation. It creates one zero-velocity
request above the observed request high-water mark and retries that immutable request
until a matching, fault-free, confirmed state arrives. Only then may live code call
`Controller::recover` with the fresh mapped state. `Adapter::reset` is a full offline
or process restart and forgets request identity; it is not live fault recovery.

## Startup compatibility

Construct the adapter with `current_chassis::Parameters` populated from the current
chassis configuration. Startup fails if shared geometry, update rate, wheel limits,
steering limits/rates/tolerances, body caps, acceleration, transition timing or stop
threshold differ. The MPPI command lifetime must not exceed the chassis timeout.
Sampling, critics, path handling, collision settings and other planner-only values
are intentionally absent from this comparison.

The adapter establishes message and nominal-parameter compatibility only. Physical
servo delay, contact, slip and stopping distance remain stage-5 measurements.
