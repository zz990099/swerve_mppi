# Validation

CMake/CTest checks kinematics, transition prediction, optimization, path progress,
nominal stopping/constraints, planning budgets, configuration, public commands and
feedback diagnostics. Fifteen closed-loop scenarios each run five seeds, including
straight, lateral, curved, reverse, spin, final yaw, corners, loops, duplicate paths,
near obstacles and translation starting in Spin.

The test-only nominal chassis compiles body intents and supplies measured mode
acknowledgements. Encoder integration is independent of predicted pose/twist. Its
shared nominal target model means these regressions preserve core behavior; they
do not validate the current Python chassis or Gazebo physics. The independent
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

CI runs Debug and Release with `-Werror`, pinned ROS formatting, installed-consumer
checks and selected UBSan regressions. It records planner performance and independent
motion-probe artifacts. It does not check out a companion repository or test removed
transport/profile paths. Historical execution results remain in Git history and do
not establish current readiness. Consult the workflow for the exact reviewed commit.
