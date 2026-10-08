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
assumption, not a reconstruction of Python's hidden `limited` velocity. Stage 3 must
reconcile issued command history, application uncertainty and observation age rather
than pretending cold seeds are exact.

Pose prediction holds the previous nominal encoder target until the next chassis
update, then applies the new target. SE(2) integration is exact per held interval;
swept-path margins enclose intermediate curved/reversal motion. This model does not
claim affine servo tracking, no-slip contact or physical stopping accuracy. Fractional
prediction periods are divided into bounded substeps; parity is checked at the real
100 Hz chassis cadence. Timer jitter/clock/freshness policy remains stage-3 work.

`check_feedback` reports configurable encoder/body disagreement.
`check_model_feedback` currently requires nominal agreement to numerical precision.
`MotionObserver` independently assesses a timestamped body observation, bounded
uncertainty and encoder-only versus independently observed stopping. It is diagnostic;
it neither loosens planning admission nor establishes a physical braking guarantee.
The strict numerical planning gate remains pending stage-3 replacement.

`FeedbackAdapter` maps named encoders into core wheel order and units. Its existing
seconds and integer-nanosecond entry points and exact snapshot/application matching
remain pending stage-3 replacement. Do not adapt asynchronous data by rewriting its
time. The adapter currently derives velocity from encoders; that identity does not
prove absence of physical slip.

With benchmarks enabled:

```bash
./build/swerve_mppi_model_probe --print-config > resolved.conf
./build/swerve_mppi_model_probe --config resolved.conf > calls.csv
python3 tools/summarize_motion_probe.py calls.csv
```

The 720 samples span three modes, six nominal/lag/slip/noise/delay conditions and
forty ticks. Independent numerical plant integration exposes model error and body
motion that stationary encoders can miss. The probe does not execute ROS/Gazebo.
