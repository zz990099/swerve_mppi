# Model and feedback boundaries

The command predictor matches the current Python chassis mechanics:

- Uniform body saturation preserves translation direction and Ackermann curvature.
- One body-intent slew fraction couples linear and angular changes; wheel and steering
  targets then approach independently at the chassis period.
- Ordinary zero immediately clears limited body intent, brakes each commanded wheel
  independently and retains commanded steering. Capture slowdown parameters do not
  govern this brake law.
- Explicit entry normalizes raw legal intent before inverse kinematics. Mechanical
  targets freeze at acceptance; nearest-angle ties match Python's candidate order.
- Large same-mode steering changes freeze a full-intent target, brake to commanded
  zero, require measured stopped wheels, align and accumulate measured alignment dwell.
  Losing alignment resets dwell. Predicted completion never acknowledges a real request.

`ChassisPrediction` is command-side simulation memory, not measured feedback or an
executor. Wheel command history uses rad/s internally to match the Python recurrence;
`VehicleState`, model wheel outputs and `Output` retain their documented SI units.
Rollouts retain memory through their horizon. A cold `DriveModel::step` seeds commanded
joints and limited body velocity from measured joints/FK. That is an explicit nominal
assumption, not a reconstruction of Python's hidden `limited` velocity. The controller
retains issued command history. Exact reported application is propagated and checked
against measured joints before its prediction memory is reused. Missing or bounded-
uncertain application remains explicit and cold-seeded; invalid history or divergence
latches a fault.

Pose prediction holds the previous nominal encoder target until the next chassis
update, then applies the new target. SE(2) integration is exact per held interval;
swept-path margins enclose intermediate curved/reversal motion. This model does not
claim affine servo tracking, no-slip contact or physical stopping accuracy. Fractional
prediction periods are divided into bounded substeps; parity is checked at the real
100 Hz chassis cadence. Source time is integer nanoseconds; planning, model and chassis
periods are separate, and warm starts advance by elapsed model intervals.

`check_feedback` reports configurable encoder/body disagreement.
`check_model_feedback` currently requires nominal agreement to numerical precision.
`MotionObserver` independently assesses a timestamped body observation, bounded
uncertainty and encoder-only versus independently observed stopping. The strict
encoder-derived numerical model gate remains unchanged. Independent motion is a
separate admission policy: accepted residual bounds grow the hard trajectory margin
over time, while exceeded envelopes and hidden motion at an encoder stop are rejected.
This still does not establish a physical braking guarantee.

`FeedbackAdapter` maps named encoders into core wheel order and units. It has one
integer-nanosecond ingress. Joint, pose and mode samples pair within configured skew,
age and future bounds; application time is not part of snapshot construction. The
adapter derives nominal velocity from encoders, so that identity still does not prove
absence of physical slip.

The higher current-chassis adapter consumes the existing chassis state and encoder
odometry DTOs. It recomputes velocity from raw wheel `rad/s` plus measured steering,
requires both published twists to agree, carries the immutable accepted request and
derives mode age only from a continuous confirmed sequence. It does not replace the
generic named-joint adapter or add a second controller ingress.

With benchmarks enabled:

```bash
./build/swerve_mppi_model_probe --print-config > resolved.conf
./build/swerve_mppi_model_probe --config resolved.conf > calls.csv
python3 tools/summarize_motion_probe.py calls.csv
```

The 720 samples span three modes, six nominal/lag/slip/noise/delay conditions and
forty ticks. Independent numerical plant integration exposes model error and body
motion that stationary encoders can miss. The probe does not execute ROS/Gazebo.
