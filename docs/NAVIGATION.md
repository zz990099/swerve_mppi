# Navigation lifecycle (0.8)

## Input and task identity

Supply the complete ordered task path on every Controller::compute call, in the
same world frame as vehicle.pose. Keep geometry, heading_policy and path_id stable
while following that task. PathManager performs local pruning internally. Do not
send a newly cropped path each tick: exact geometry changes are treated as replans.

A change to any waypoint position/yaw, heading_policy or path_id restarts progress,
clears the optimizer warm start and resets goal/stall state. Increment path_id to
restart an identical completed task. Controller::reset also clears navigation
state, while preserving the execution request ID high-water mark. The controller
expects finite, strictly increasing timestamps and one call per dt_s; an adapter
must handle freshness, time resets, cancellation and localization discontinuities.

Replanning cancels obsolete same-mode alignment intent. It does **not** cancel or
rewrite an already issued RequestMode: that immutable handshake must complete or
time out under its original deadline. The next drive is planned for the new path.
For emergency cancellation use the execution SafeStop/recovery contract.

PathHeadingPolicy specifies **body yaw**, independently of path travel direction:

- FollowPath (default): interpolate waypoint yaw along arc length and penalize
  heading error throughout tracking. This is a soft cost, not a hard constraint.
- GoalOnly: ignore intermediate yaw in tracking costs and require only final yaw.

Waypoint order defines travel order, including reverse travel and foldbacks. Do
not reverse the point array to request reverse gear; specify positions behind the
robot while keeping the desired body yaw. A zero-length path requests final yaw
alignment. Consecutive duplicate positions are allowed; they do not encode a
mandatory intermediate spin or stop. Final yaw is retained.

Controller fills TrackingContext after pruning. Callers should leave input.tracking
empty; a standalone Optimizer caller may supply a validated local context with a
nonnegative speed cap. All supplied fields must be finite even when Controller
will replace them.

## Path progress

PathManager stores cumulative arc length and the current segment. Initial matching
is anchored to the first nonzero segment, bounded by path_search_window_m. Later
segments become eligible in waypoint order after measured motion captures their
shared endpoint, or crosses its endpoint plane for a smooth sampling point.
Sharp corners/reversals require XY capture within position tolerance by measured
XY or the segment between successive measurements. Smooth samples are not mandatory
precision waypoints. This permits multiple dense waypoints per tick without
selecting a nearby return leg at a crossing, foldback or short loop.

Progress never decreases within one task. Subsequent forward intervals are limited
to the smaller of path_search_window_m and measured displacement plus
path_progress_slack_m. This is not global relocalization or unrestricted loop matching.
Start near the beginning of the task path. For a major localization jump, submit
a new task path beginning near the corrected pose.

The local target lies path_lookahead_m ahead in arc length. Accumulated geometric
turn greater than path_lookahead_turn_rad truncates the target at the corner, so
lookahead cannot point behind a not-yet-reached reversal. A corner is captured
only when both the arc gap and measured XY gap are within position tolerance.
Within goal_docking_distance_m, a bounded low-speed translation policy finishes
corner capture. Intermediate corners never trigger task completion.

The path heading uses shortest-angle interpolation. Cross-track output is the XY
distance to the matched monotonic progress point, not a global nearest-distance
query. Progress and remaining length are diagnostics, not obstacle clearances.
Geometry scans now start at progress via binary lookup and stop at the bounded
window. Validation and path identity checks still scan the full input, and the
local reference is copied each tick. Allocation-free operation remains future
work; docs/PERFORMANCE.md includes dense-path profiling.

PathReference exposes the effective translation target in target, its
PathTargetKind (Lookahead, Corner or Goal), target_remaining_m (arc gap to that
target), and goal_eligible. Legacy terminal/corner_target flags describe the local
reference. PathManager grants goal_eligible only when the local reference reaches
the global goal without an uncaptured blocking corner. Controller and GoalManager
both require this qualification for terminal takeover/initial position acquisition.
Standalone GoalManager callers must explicitly provide goal_eligible for a terminal
reference; a default PathReference does not authorize completion.

## Terminal control and completion

Within goal_slowdown_distance_m in arc length of the effective corner or eligible
goal, sampled, nominal and weighted controls obey a translation speed cap based
on distance to that target, a proportional gain and a stopping-distance bound.
Uncaptured corners take precedence over a nearby global endpoint, so short loops
and foldbacks can start even when the global goal coincides with the robot.
Lookahead targets retain normal limits. Tracking costs add path-heading, command-change and
terminal-speed penalties. The optimizer reserves a fresh geometric seed alongside
the nominal and Gaussian proposals to adapt to moving local targets.

With terminal eligibility and within goal_docking_distance_m of both the path end and goal position, Controller
uses deterministic low-speed capture. It retains straight DualAckermann motion
when the lateral error is small; otherwise it uses Crab translation. After position
capture it brakes and requests Spin when final yaw requires correction. Switching
requires measured stopping and minimum mode dwell. Positive drive and new requests
are checked through alignment/entry, the next Drive and its full stopping
continuation by the shared hard validator. Subsequent controls in this check are
zero, so it does not extrapolate the proportional command past its target.
Terminal control is a proportional capture policy, not an additional MPPI solve.

NavigationStatus is separate from the execution TransitionPhase:

| Status | Meaning |
| --- | --- |
| Tracking | Following the ordered path |
| ApproachingGoal | Remaining arc length is within slowdown distance |
| AligningGoal | Final position acquired; yaw still outside tolerance |
| Settling | Position acquired and yaw in tolerance; waiting for strict pose/stop dwell |
| Complete | Measured completion latched for this task |
| Fault | Controller emitted SafeStop |
| Waiting | No feasible motion; checked Brake/Hold awaiting fresh input |

Completion requires remaining arc length and position capture, actual XY/yaw
within their tolerances, confirmed fault-free mode, stopped body **and every wheel**,
no active execution/alignment, and a continuous goal_settle_time_s dwell. Being
near the endpoint of a closed loop at startup cannot complete it. Position capture
uses hysteresis while correcting yaw; completion always uses strict tolerances.
Repeated/nonincreasing samples or a gap greater than 1.5 * dt_s restart the
stopped dwell. TimedExecutor enforces configured cadence and freshness before
transported commands reach execution. GoalManager alone cannot verify feedback
age relative to an independent current clock.

Output::goal_reached latches after completion until replan/reset. The controller
then emits Hold (or Brake if external motion is measured), without stochastic
resampling. The latch records task success; it is not a continuously recomputed
pose-in-tolerance flag and does not automatically drive back after external drift.

Output also reports path_progress_m, remaining_path_m, cross_track_error_m,
goal_distance_m, goal_yaw_error_rad and stalled. The stall timer observes arc
advance, goal-distance reduction or terminal yaw improvement. Active handshakes
and local committed alignment use their own execution timeout. Stable stopping
and completed tasks do not report a stall; wheels that never stop can. stalled
is a diagnostic only: recovery policy belongs to the caller.

NoFeasiblePlan returns Waiting/Blocked with Brake/Hold only when a complete
predicted stop passes hard validation. Fresh inputs are replanned without executor
reset. UnsafeStoppingTrajectory and execution faults emit latched SafeStop;
removing an obstacle alone does not recover those faults. A checked planning stop
discards local alignment/warm start; an active RequestMode remains immutable.
All normal Brake/Hold/RequestMode outputs also pass a current-constraint stopping
check. Normal goal stops retain AligningGoal/Settling/Complete; a rejected stop
reports UnsafeStoppingTrajectory and Fault instead of claiming successful capture.

## New configuration defaults

| Parameter | Default | Purpose |
| --- | --- | --- |
| path_lookahead_m | 1.0 m | Arc-length local target |
| path_lookahead_turn_rad | 1.0 rad | Corner lookahead cutoff |
| path_search_window_m | 1.5 m | Maximum forward match window |
| path_progress_slack_m | 0.1 m | Additional match allowance per tick |
| goal_position_tolerance_m | 0.06 m | Position/corner capture tolerance |
| goal_yaw_tolerance_rad | 0.05 rad | Final body yaw tolerance |
| goal_settle_time_s | 0.3 s | Continuous measured-stop dwell |
| goal_slowdown_distance_m | 0.6 m | Start terminal translation cap/cost |
| goal_docking_distance_m | 0.25 m | Start deterministic capture |
| goal_translation_gain | 1.2 /s | Position correction gain |
| goal_rotation_gain | 1.5 /s | Yaw correction gain |
| progress_timeout_s | 3.0 s | No-progress diagnostic interval |
| progress_distance_m | 0.03 m | Significant progress threshold |
| path_heading_weight | 0.3 | Per-tick body-yaw tracking cost |
| smoothness_weight | 0.08 | Squared active command differences |
| goal_speed_weight | 2.0 | Terminal squared translation speed |

The existing stopped thresholds, rate limits and execution deadlines still apply.
These gains and geometric tolerances are standalone defaults, not calibrated
hardware settings. Version 0.5 adds public fields and changes task semantics;
rebuild downstream users and request find_package(swerve_mppi 0.5 CONFIG REQUIRED).
The version 0.4 joint-target execution contract remains in effect.

Version 0.6 retains these navigation defaults and adds computation policy, failure
reason and planning-work diagnostics. Rebuild consumers against the 0.6 package;
see PERFORMANCE.md for the workspace ownership and diagnostic contracts.

Version 0.8 adds explicit navigation targets and goal eligibility, and validates
all controlled stops. Rebuild consumers with find_package(swerve_mppi 0.8 CONFIG REQUIRED).
