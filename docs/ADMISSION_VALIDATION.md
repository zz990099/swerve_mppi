# Feedback, workload and profile admission (0.16)

This release addresses the remaining standalone review cases: odometry reporting
0.8 m/s while all encoders report zero, an unbounded 500-obstacle solve, and an
adapter retaining or reshaping endpoint commands instead of sampling the checked
joint interval. Full ROS transport and physical model calibration are separate work.

## Admission and resource limits

`check_feedback` reports the linear vector norm and angular absolute discrepancy
between body-frame velocity and wheel forward kinematics. Its configurable diagnostic
defaults remain 0.05 m/s and 0.10 rad/s, inclusive with a 1e-9 numerical allowance.
A Valid diagnostic result alone does **not** authorize a nominal model or a stop.

All normal modelling, planning, stopping validation, mode supervision, execution
and stopped recovery use `check_model_feedback`. This additionally requires each
error norm to be at most 1e-9, allowing roundoff only. Larger disagreement returns
InconsistentFeedback/SafeStop without a normal braking certificate, including a
0.04 m/s measured body speed with stationary wheels. Moving encoders do not exempt
additional body motion. No measured twist is overwritten to make a sample fit.

The nominal model has no uncertainty envelope for slip, estimator error or delay.
Integrating independent odometry requires an explicit robust motion/uncertainty
contract before relaxing model admission; changing diagnostic tolerances does not
relax it. Encoder-derived odometry agrees by construction and is not slip validation.

| Parameter | Default | Accepted configuration |
| --- | --- | --- |
| max_path_points | 4096 | 1–65536 |
| max_obstacles | 128 | 1–4096 |
| horizon_steps | 20 | 2–512 |
| stopping_horizon_steps | 200 | 2–4096 |
| samples_per_branch | 80 | 1–2048 |
| iterations | 2 | 1–32 |
| compute_budget_ratio | 0.8 | 0–1; zero is offline-only |

Input caps are checked before scanning or copying contexts. WorkloadExceeded
rejects excess; paths and obstacles are never truncated. Shared validators must
use identical wheelbase/track, admission tolerances, footprint, joint limits and
input caps. Workspace caps bound allocation
dimensions, not execution time; raising them requires target-host measurement.

PlanningBudget uses steady_clock, independent of simulation time. One budget spans
Controller preparation, every mode branch, stopping checks and the final output
gate. Optimizer cooperatively checks between evaluations; expiry discards the
solution and clears the warm start without reseeding noise. Controller emits
ComputeTimeout/SafeStop,
clears drive/request/completion output and marks PlanningStats::budget_exhausted.
Expiry rounds conservatively at the nanosecond boundary. One rollout or arbitrary
custom critic can overrun the budget; callbacks must be bounded. This is late-result
rejection, not a hard real-time scheduling guarantee. An independently scheduled
actuator callback and endpoint watchdog must cover delayed or silent planners.

## Profile consumer

ProfileRunner accepts only checked TimedExecutor results with compatible actuator
configuration and an aligned start stamp. Checked CommandRejected/TaskMismatch
stopping fallbacks remain executable, including RequestMode alignment from an
already committed transition. The request identity, entry geometry and fixed deadline
remain owned by ModeExecutor; a rejected Drive never becomes executable. It samples
the existing ActuationPlan rather
than inferring a mode, recomputing ramps or duplicating the mode state machine.
Wheel linear m/s become joint rad/s using wheel_radius_m. Missed tick boundaries,
expired profiles, replayed start times, wall-clock expiry during a paused world and
either clock rollback latch failure. No target requires independent emergency stop.
Verified stopped/confirmed recovery plus a new TimedExecutor session is required;
serialize all calls and drain stale transport commands. The helper cannot detect a
dead process without being called and is not a complete ROS node.

Gazebo's external_joint_control option disables internal motion supervision,
cmd_vel handling and both joint command publishers. It retains encoder odometry
and feedback-health diagnostics, leaving mode feedback to the external executor.
ros2_control forward command controllers still require an external watchdog.

## Regression evidence

admission_regressions tests the reviewed 0.8 m/s and 0.04 m/s/zero-wheel cases, opposite
body/joint disagreement, angular and frame-direction errors, inclusive tolerances,
numerical model admission, and rejection through standalone models, Controller,
ModeManager, ModeExecutor,
TimedExecutor and caller-supplied trajectory validation. A fake monotonic clock
advances inside a hard constraint to verify shared branch budgets, exact expiry,
already expired optimization and late terminal Hold rejection without host timing.
Workload cases cover 500 obstacles, oversized paths, configuration caps and inclusive
input boundaries. Actual guarded profiles cover interpolation/units, exact endpoint,
replacement, checked stopping fallback, missed boundaries, replay, paused-clock
watchdog, rollback and fault-preserving moving recovery. The installed consumer
links the new public APIs. Pending transition task/command rejection cases sample
checked zero-drive alignment to stopped confirmation, retain ID/entry geometry and
verify stalled retries cannot renew the deadline. All four validator consumers reject
each foreign geometry dimension; matching custom geometry validates a spinning stop.
A fixed-seed narrow feasible constraint initially rejects all proposals, then admits
Drive on a later retry. Explicit reset reproduces the complete recovery sequence.
optimizer_regressions also compares warm-start clearing against a continuing
nonzero-noise stream and verifies explicit optimizer replay. Functional
algorithm/behavior tests disable wall budgets
explicitly; production defaults remain enabled.

These checks do not establish physical stopping distance or validate MPPI tracking
in Gazebo. Compare measured joint tracking and independent Gazebo truth against
the nominal affine/braking curves before accepting model collision guarantees.
