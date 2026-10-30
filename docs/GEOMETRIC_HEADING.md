# XY path following with independent final orientation

External paths supply ordered XY geometry and one final pose yaw. Intermediate
yaw and coincident-position rotation markers are ignored. `OrderedPath::set`
prepares the complete route before cropping; the original final pose is kept
separately. `lookahead` (0.30 m) derives forward heading without looking through
mandatory corners. Pure rotation tasks remain supported by the final pose yaw.

`Input::reference_kind` distinguishes external XY paths, geometric paths,
controller-prepared `EffectivePath`, fixed internal heading, wheel alignment,
and departure. The controller constructs one effective yaw path from travel
geometry and independent final yaw. A spatially decaying startup offset avoids
an instantaneous body-yaw step. All consumers, including the wheel/velocity
profile, seed, critics, recovery and retiming, use this effective path.

Final orientation uses measured ordered remaining distance and XY goal distance.
All samples share one smoothstep weight per cycle, starting at 0.50 m and reaching
full authority at 0.10 m. There is one circular heading target and one heading
error; its scale changes from `travel_heading_scale=0.50` rad to
`goal_heading_scale=0.10` rad. The old duplicate goal/path heading objectives
and ordinary yaw-corridor slack are removed. Position tracking remains stronger.
The former predicted-remaining 0.02 m switch remains removed.

The common velocity profile reserves time for final rotation. Once measured
ordered progress and XY position enter the goal region, translation correction
stops while final yaw is corrected. Core and ROS share `GoalTolerance`; the
adapter passes its actual OrderedGoalChecker settings. A stopped arrived state
clears obsolete steering intent and produces a slew-limited, rolled-out,
critic-validated zero request. The measured wheel-speed, settling, zero-output,
feedback-health and collision gates remain active. Precise internal maneuvers
keep their own references and tighter alignment behavior.

Effective reference snapshots include completion tolerances for retiming.
Synthetic tests with explicit internal yaw select `EffectivePath`; this is not
a ROS switch for restoring external intermediate yaw semantics.

## Validation

`heading_tests` checks intermediate-yaw invariance, final yaw retention, corner
preview, duplicate-point handling, measured terminal weighting, off-path and
ordered activation guards, pure rotation and retiming snapshot retention.
Existing synthetic actuator/branch tests explicitly use prepared references.
Physical recordings separate normal travel heading metrics from terminal and
internal alignment; positional and final-yaw acceptance thresholds are unchanged.

The physical workflow remains one 32-case Gazebo/Nav2 round with seed 42 and
minimum route length 12 m. No real-time acceptance gates are relaxed.

`record_portable` can run one selected scenario or an explicit full round,
with configurable route length, speed and independent steering servo response.
The default Python entry selects `straight`, not all 32 cases. It records failures
and partial trajectories. See [local reproduction](LOCAL_TESTING.md).

The implementation remains experimental. The latest seven-case local round
passes functional completion and the 10 cm XY corridor, but strict heading and
180-degree final-turn regressions remain. See
[latest implementation and limitations](GEOMETRIC_HEADING_LOCAL_OPTIMIZATION.md).
Historical numerical and Gazebo evidence remains in the earlier validation reports;
it does not validate this new revision.
