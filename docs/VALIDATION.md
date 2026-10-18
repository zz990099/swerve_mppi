# Validation

## Local checks

Standalone core tests cover inverse/forward kinematics, mode constraints, actuator
slew, nominal switching, invalid inputs, injected critic rejection, grid geometry,
computing again after an invalid call, and projected/predicted wheel-speed limits
in every mode with both command signs. An installed-consumer test relocates the
install prefix before finding/linking the public CMake package.

Caller helpers also build without ROS. They check frozen mode requests, ordinary
drive receipt preservation, faulted-start zero initialization, ordered paths,
external goal settling and injected map costs.

```bash
cmake -S swerve_mppi_core -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install-core"
cmake -S swerve_mppi_ros/tests/portable -B build-support \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/install-core" \
  -DCHASSIS_SOURCE_DIR=/absolute/path/to/swerve_gazebo_sim
cmake --build build-support --parallel 2
ctest --test-dir build-support --output-on-failure
```

The optional Python integration test runs the compiled controller, actual ROS-side
adapter and external GoalChecker against the companion production Python `Chassis`.
It integrates encoder motion outside core and tests straight, crab, reverse, spin,
terminal yaw, curved, out-and-back, closed-loop and wheel-limited spin paths, both
ideal actuator following and 30 ms actuator lag. This is eighteen command-loop
scenarios, not independent physical validation. Returning paths must visit their
intermediate checkpoints in order before completing; every command must satisfy
the configured wheel-speed bound. Each
must reach 0.06 m / 0.05 rad with measured speed below 0.035 m/s / rad/s for 0.3 s,
within 50 simulated seconds. No per-speed-command receipt is produced.

## ROS and physical acceptance

Native Jazzy tests load both plugins via pluginlib and exercise lifecycle,
absolute/percentage speed limits and reset, missing TF/stale state, injected
costmap rejection, frozen mode handshakes, final-velocity bridge stop authority,
and ordered GoalChecker settling. CI launches a fresh Harmonic chassis with the
actual Nav2 controller server for each of eight FollowPath action cases, including
out-and-back and closed-loop tasks with ordered checkpoint assertions.
Independent Gazebo truth is read only by the test. GoalChecker uses the normal
encoder estimate; the physical test additionally requires truth error below 0.15 m
and 0.10 rad at completion. Each case starts a fresh world and encoder origin;
this isolates single-task tracking from accumulated localization drift. These are
first-stage acceptance thresholds, not claims
about a calibrated real robot. Logs and native ROS test results are CI artifacts.

```bash
ros2 launch swerve_mppi_ros demo.launch.py headless:=true publish_ground_truth:=true
# Another sourced terminal:
python3 swerve_mppi_ros/tests/gazebo_tracking.py \
  --chassis-root /absolute/path/to/swerve_gazebo_sim
```

A sequential six-task experiment on 2026-10-17 reached every encoder-defined goal,
but ended with 0.106 m / 0.109 rad absolute truth error, exceeding the 0.10 rad
heading threshold. Independent-task acceptance does not establish long-duration
localization accuracy. Keep the no-`--scenario` form of `gazebo_tracking.py` for
repeating that separate cumulative-drift experiment.

## Remaining work

1. Establish reproducible physical tracking/settling results and tune the nominal
   response from measured traces, including same-mode alignment pauses.
2. Validate map critic behavior in static-obstacle scenes; add footprint-aware
   scoring when center costs in an inflated map are insufficient.
3. Identify real hardware response and state-estimation behavior before real-robot
   acceptance. Measure compute timing on the intended onboard computer.
4. Expand Nav2 integration to sensor-populated obstacle maps and a complete
   navigation bringup; the supplied launch intentionally tests FollowPath only.

Pre-0.28 tests and execution-evidence contracts were removed with their production
implementations. Their old results do not establish acceptance of this controller.
