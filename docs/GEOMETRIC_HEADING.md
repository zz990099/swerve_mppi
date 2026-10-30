# XY path following with independent final orientation

External paths supply ordered XY geometry and one final pose yaw. Intermediate
yaw and coincident-position rotation markers are ignored. `OrderedPath::set`
prepares the complete route before cropping; the original final pose is kept
separately. `lookahead` (0.30 m) derives forward heading without looking through
mandatory corners. Pure rotation tasks remain supported by the final pose yaw.

`Input::reference_kind` distinguishes external paths, prepared geometric paths,
fixed internal heading, wheel alignment, and departure. Low-level callers passing
already-prepared references must explicitly select `GeometricPath`; ordinary
external input defaults to `ExternalPath`. The internal `OrderedPath::set`
reference-kind argument retains explicit ordered rotation tests and is not a
ROS configuration switch for restoring intermediate external yaw semantics.

Final orientation activates using measured ordered remaining distance and XY
distance to the final goal. All samples share one smoothstep weight per cycle,
starting at 0.50 m and reaching full authority at 0.10 m. Travel heading cost and
heading-corridor penalties fade together. Goal heading is scored throughout each
prediction; the former candidate-dependent 0.02 m gate is removed. Feedback seed
and latched steering intent use the same weighted yaw objectives. The goal cost
scale is 0.35 rad, independently of completion tolerances and precise internal
heading alignment. The geometric nominal speed envelope is retained; all blended
requests still undergo conditioning and actuator rollout feasibility checks.

Internal branch maneuvers preserve their fixed heading/departure intent and
precise alignment semantics. Effective references, reference kind, ordered
remaining distance and blend weight are retained for publication retiming even
during normal tracking. Ordinary goal blending does not reset the optimizer on
every cycle. Existing terminal hold, zero-output completion readiness, live
map-to-odom goal conversion, feedback health and settling gates remain enabled.

## Validation

`heading_tests` checks intermediate-yaw invariance, final yaw retention, corner
preview, duplicate-point handling, measured terminal weighting, off-path and
ordered activation guards, pure rotation and retiming snapshot retention.
Existing synthetic actuator/branch tests explicitly use prepared references.
Physical recordings separate normal travel heading metrics from terminal and
internal alignment; positional and final-yaw acceptance thresholds are unchanged.

The physical workflow remains one 32-case Gazebo/Nav2 round with seed 42 and
minimum route length 12 m. No real-time acceptance gates are relaxed.

`record_portable` records the same 32 scenario shapes at their original lengths,
with seed 42, 20 Hz requests, the shared chassis executor and an independent
0.12 s steering servo plant (controller nominal 0.10 s). It retains partial
trajectories and reports stalled/no-solution/deviation cases without tuning them
away. Numerical results do not validate ROS lifecycle, TF, odometry or Gazebo.
Run it with a directory containing `cases.txt` and `references/<name>.csv`.

This implementation is experimental until the complete regression and physical
results have been assessed. Known failures are retained in the validation report.

Numerical reproduction after building the portable target:

```bash
python3 swerve_mppi_ros/tests/run_portable_recordings.py \
  --executable build-support/record_portable --output heading-results
```

The numerical montage uses functional completion and a 0.10 m position corridor;
it is not a substitute for the existing full physical quality verdict.
