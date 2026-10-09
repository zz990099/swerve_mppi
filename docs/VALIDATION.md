# Validation

Obstacle clearance uses the nearest clearance per trajectory segment, independent
of repeated obstacle sampling. A regression verifies duplicate samples do not change
soft cost, while collision rejection remains a hard check.

CMake/CTest checks kinematics, transition prediction, optimization, path progress,
nominal stopping/constraints, planning budgets, configuration, public commands and
feedback diagnostics. Dedicated timing/lifecycle regressions cover bounded source
pairing, stale/future samples, publication/expiry, exact and uncertain application,
history divergence, fault latching/recovery and required independent motion.
Fifteen closed-loop scenarios each run five seeds, including
straight, lateral, curved, reverse, spin, final yaw, corners, loops, duplicate paths,
near obstacles and translation starting in Spin.

The test-only nominal chassis compiles body intents and supplies measured mode
acknowledgements. Encoder integration is independent of predicted pose/twist. Its
shared nominal target model means these regressions preserve core behavior; they
do not independently validate command generation or Gazebo physics. The independent
720-sample motion probe exposes lag, slip, noise and delayed observations separately.

The installed-consumer test builds against a fresh install prefix, compiles every
public header independently, verifies the body-command API and rejects installation
of removed execution APIs or private prediction headers. It has no source include
path. Optional benchmark tests check allocation bounds and serialized motion data.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSWERVE_MPPI_BUILD_BENCHMARKS=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure --parallel 2
```

Enable the direct Python/core comparison with:

```bash
cmake -S . -B build -DSWERVE_MPPI_CHASSIS_SOURCE_DIR=/absolute/path/to/swerve_gazebo_sim
```

`chassis_python_parity` runs the actual companion Python class and compares 7,960
command cycles against the C++ predictor, including all six directed mode changes,
two wheel limit/ramp sets, signed reversals, saturation, nearest-angle ties, normal
zero and automatic realignment. It checks every joint target, limited body velocity,
frozen geometry and transition phase. Both implementations consume the same measured
sample and actual elapsed interval. Separate core tests hold measured feedback back
to verify stop-before-align and dwell reset. These tests deliberately exclude ROS,
watchdogs and physical Gazebo servo/contact dynamics.

CI runs Debug and Release with `-Werror`, pinned ROS formatting, installed-consumer
checks and selected UBSan regressions. It records planner performance and independent
motion-probe artifacts. It pins companion commit
`9358c96884d7fa4866cea7c053daa7a7cec82d8f` for offline Python parity. It does not run
Gazebo or test removed transport/profile paths. Historical execution results remain
in Git history and do not establish current readiness. Consult the workflow for the
exact reviewed commit.
