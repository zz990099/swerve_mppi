# Model and feedback boundaries

The current nominal model uses affine drive-joint interpolation and proportional
braking before steering alignment. Its rollout and stopping checks are conditional
on that model. Stage 2 replaces the behavior assumptions needed to match the current
Python chassis; servo lag, contact and slip still require physical validation.

`check_feedback` reports configurable encoder/body disagreement.
`check_model_feedback` currently requires nominal agreement to numerical precision.
`MotionObserver` independently assesses a timestamped body observation, bounded
uncertainty and encoder-only versus independently observed stopping. It is diagnostic;
it neither loosens planning admission nor establishes a physical braking guarantee.
The strict nominal gate remains unchanged in stage 1.

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
