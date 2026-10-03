# Accepted entry feedback and numerical boundaries (0.20.0)

Review baseline: 4638cb798986a21c5f4698a7283087ee7f3f3c84 (0.19.1).
Scope: the three reviewed portable-core findings. ROS/Gazebo physical tracking
acceptance remains integration work.

## Frozen geometry follows execution acceptance

The 0.19.1 equivalent-angle acknowledgement fix did not update the geometry used
by pending-request stopping checks. A lateral request planned from +0.01 rad can
predict +pi/2 while first queued execution from -0.01 rad accepts -pi/2. With a
shared const hard constraint permitting steering away from zero, that actual
negative interval is valid, but checking the stale positive interval withheld the
controller command with UnsafeStoppingTrajectory.

ModeExecutor now echoes its complete first-accepted JointModeRequest in
ModeFeedback::accepted_mode_request, retained through completion and normal
operation. FeedbackAdapter and actuator endpoint/independent encoder fixtures
copy it into VehicleState. Hypothetical transition rollouts clear the real receipt
rather than fabricate executor acceptance.

Before acceptance, ModeManager predicts geometry from the latest snapshot using
the frozen body intent. A matching accepted ID requires a receipt with identical
mode/body payload and physically equivalent rolling lines (modulo pi, 1e-7 rad
numeric allowance). The planner then binds the exact accepted mechanical targets
for every retry safety check. Missing or changed bound receipts fault. Final
handover requires stopped, confirmed, matching mode/ID and measured alignment to
those exact targets within steering_tolerance_rad. Moving measured angles never
redefine the commitment. Accepted IDs, deadlines and public body requests retain
their existing immutability rules.

Chassis regressions run both signed lateral goals through Controller, TimedExecutor
and independent high-rate ProfileRunner/encoder consumption with the directional
joint constraint. They cover changed snapshots before acceptance, opposite signed
acceptance, healthy retries, first Drive and completion. Additional guards reject
missing, wrong-ID, wrong-mode, changed-body, changed-geometry, unbounded and nonfinite
receipt data, both at initial binding and after acceptance. Valid measurement
changes retain the frozen geometry; equivalent measured lines alone cannot complete
handover to a different accepted mechanical representation.

## Entry direction is independent of amplitude

Previously body and per-wheel 1e-9 thresholds disagreed for Spin yaw rates near
1e-9 rad/s. Inverse kinematics retained straight steering for tiny wheel vectors,
then ModeExecutor rejected that geometry as a Spin entry.

steering_for_entry now divides the projected nonzero intent by its largest absolute
component before inverse kinematics. Direction/curvature sets geometry, independent
of speed amplitude. Division avoids reciprocal overflow for subnormal inputs.
Exactly zero retains canonical mode geometry. The original body velocity is never
rescaled in the request or receipt. Ordinary rolling dynamics retain their own
rate/stop semantics.

Public TimedExecutor regressions cover Spin, Crab and curved DualAckermann over
positive scales 1, 1e-8, 5e-9, 1e-9, 1e-10 and 1e-300, plus both signed smallest
subnormal Spin intents. They require healthy checked profiles, accepted receipts
and amplitude-independent steering positions.

## Finite unwrapped yaw and derived errors

Subtracting opposite finite extreme headings overflowed before wrapping, causing
NaN yaw diagnostics and an indefinite zero-command settling state. angle_distance
now reduces each operand before subtraction. Body/world transforms, constant/ramp
and moving-steering integration, path interpolation and path-heading scoring also
reduce the initial heading before local increments. Already bounded angles use a
fast path. No arbitrary restriction to +/-pi is imposed on finite input yaw.

GoalManager rejects nonfinite derived distance/yaw errors before mutating its
settling observation clock. Controller returns absent authorization and InvalidInput
with finite default diagnostics. Tests compare DBL_MAX and -DBL_MAX inputs with
their periodic representations through public planning, path interpolation, finite
critic scoring and fixed/moving steering/braking integration. Finite extreme XY
subtraction overflow separately verifies derived-error rejection.

## Migration

Version 0.20 changes public ModeFeedback and VehicleState aggregate layout. Rebuild
all consumers. Forward the complete optional accepted request unchanged with the
executor mode status; FeedbackAdapter does this automatically. An accepted pending
ID without its receipt is insufficient. Receipt absence is normal at startup and
after deliberate executor recovery; request-ID high-water marks still survive
reset. A new accepted request replaces the previous receipt. The public MPPI output
remains actionless chassis velocity, explicit mode and optional body mode request.

See CHASSIS_COMMAND.md, EXECUTION_CONTRACT.md and SIMULATION_PREPARATION.md for the
updated mapping, lifecycle and adapter requirements.

## Verification

Final local validation used GCC 13.3, C++17 and -Werror:

- Release: all 56 CTests passed (106.33 s).
- Debug: all 56 CTests passed (383.65 s).
- Both runs include pinned official ROS Rolling formatting, installed-package
  consumption and independent compilation of all 19 public headers, the new
  acceptance/numerical regressions and existing raw/timed/profile multi-seed loops.
- Final official-format and git diff --check verification passed.

After both runs completed, the Release production integration-budget probe ran
50 cold-start repetitions per obstacle count with the default 80 ms budget:

| Obstacles | p50 (ms) | p95 (ms) | Maximum (ms) | Compute timeouts | Total overruns | Other failures |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 35.5430 | 42.4732 | 50.7797 | 0 | 0 | 0 |
| 40 | 43.9984 | 47.0668 | 49.2560 | 0 | 0 | 0 |
| 128 | 67.0269 | 74.1453 | 80.0666 | 2 | 2 | 0 |

At 128 obstacles, two of 50 attempts hit the planning deadline and withheld
command authorization as designed; the maximum complete integration cycle exceeded
80 ms by about 0.067 ms. This measurement does not establish enough production
headroom for the default 128-obstacle workload on this host. The previous release's
small measured margin was not a robust timing guarantee either. Target-machine
measurement and selection of workload/budget remain required before continuous
simulation or hardware operation. Functional tests intentionally disable the
production wall-time budget and test deadline behavior separately; passing those
tests does not remove this measured performance limit.
