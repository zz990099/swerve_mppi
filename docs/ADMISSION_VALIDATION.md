# Feedback, workload and profile admission (0.15)

This release addresses the remaining standalone review cases: odometry reporting
0.8 m/s while all encoders report zero, an unbounded 500-obstacle solve, and an
adapter retaining or reshaping endpoint commands instead of sampling the checked
joint interval. Full ROS transport and physical model calibration are separate work.

## Admission and resource limits

All normal modelling, planning, stopping validation and execution require finite
body-frame velocity consistent with wheel forward kinematics. FeedbackCheck reports
linear vector norm and angular absolute discrepancy. Defaults are 0.05 m/s and
0.10 rad/s, inclusive with a 1e-9 numerical allowance. InconsistentFeedback returns
SafeStop without certifying a nominal braking fallback. No observation is silently
rewritten or replaced with predicted feedback. These tolerances do not enlarge
collision margins or constitute a model of tire slip, estimator uncertainty or delay.

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
use identical admission tolerances and input caps. Workspace caps bound allocation
dimensions, not execution time; raising them requires target-host measurement.

PlanningBudget uses steady_clock, independent of simulation time. One budget spans
Controller preparation, every mode branch, stopping checks and the final output
gate. Optimizer cooperatively checks between evaluations; expiry discards the
solution and clears the warm start. Controller emits ComputeTimeout/SafeStop,
clears drive/request/completion output and marks PlanningStats::budget_exhausted.
Expiry rounds conservatively at the nanosecond boundary. One rollout or arbitrary
custom critic can overrun the budget; callbacks must be bounded. This is late-result
rejection, not a hard real-time scheduling guarantee. An independently scheduled
actuator callback and endpoint watchdog must cover delayed or silent planners.

## Profile consumer

ProfileRunner accepts only checked TimedExecutor results with compatible actuator
configuration and an aligned start stamp. Checked CommandRejected/TaskMismatch
stopping fallbacks remain executable. It samples the existing ActuationPlan rather
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

admission_regressions tests the reviewed 0.8 m/s/zero-wheel case, opposite
body/joint disagreement, angular and frame-direction errors, inclusive tolerances,
and rejection through standalone models, Controller, ModeManager, ModeExecutor,
TimedExecutor and caller-supplied trajectory validation. A fake monotonic clock
advances inside a hard constraint to verify shared branch budgets, exact expiry,
already expired optimization and late terminal Hold rejection without host timing.
Workload cases cover 500 obstacles, oversized paths, configuration caps and inclusive
input boundaries. Actual guarded profiles cover interpolation/units, exact endpoint,
replacement, checked stopping fallback, missed boundaries, replay, paused-clock
watchdog, rollback and fault-preserving moving recovery. The installed consumer
links the new public APIs. Functional algorithm/behavior tests disable wall budgets
explicitly; production defaults remain enabled.

These checks do not establish physical stopping distance or validate MPPI tracking
in Gazebo. Compare measured joint tracking and independent Gazebo truth against
the nominal affine/braking curves before accepting model collision guarantees.
