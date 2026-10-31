# Continuous Swerve MPPI

A C++17 sampling controller for four independent steering/drive modules. The
optimizer samples body-frame `(vx, vy, wz)` sequences, predicts actuator execution
using `swerve_chassis_core`, and returns a body velocity through Nav2's ordinary
FollowPath velocity chain. No discrete motion modes or mode handshake remain.

Packages:
- `swerve_mppi_core`: ROS-free optimizer, shared-chassis prediction and critics.
- `swerve_mppi_ros`: Nav2 Controller and OrderedGoalChecker plugins.

## Dependencies and build

The exact compatible chassis revision is recorded in `dependencies.repos`.
Place both repositories beneath your ROS workspace `src/`, then:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to swerve_mppi_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch swerve_mppi_ros demo.launch.py headless:=true
```

The supplied launch and physical CI target **ROS 2 Jazzy / Gazebo Harmonic**.
Humble Nav2 is not claimed supported by this plugin build: its interface/config
must be validated separately. The chassis itself has its own Humble CI.

Standalone build first installs `swerve_chassis_core`, then supplies its prefix:

```bash
cmake -S swerve_mppi_core -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/path/to/chassis/install
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

Version 1.0 intentionally removes the old mode API and changes drive speeds from
wheel-rim m/s to joint rad/s. Existing external callers must supply a valid
execution snapshot, measured joints and matching physical configuration.

[Architecture](ARCHITECTURE.md) · [ROS contract](docs/ROS.md) · [Validation](docs/VALIDATION.md)

External path intermediate yaw is ignored: travel heading follows ordered XY geometry,
with an independent final yaw. This policy is experimental; see
[heading contract](docs/GEOMETRIC_HEADING.md) and
[recorded limitations](docs/GEOMETRIC_HEADING_VALIDATION.md).

For incremental local single-scenario tests without ROS, see
[local testing](docs/LOCAL_TESTING.md). The latest experimental heading changes
and unresolved strict regressions are recorded in
[local optimization results](docs/GEOMETRIC_HEADING_LOCAL_OPTIMIZATION.md).

The current body-heading policy is a bounded critic preference, with independent
final yaw: [policy and known limits](docs/SOFT_HEADING_POLICY.md).

The frozen 32-case local round and retained failures are documented in
[soft-heading validation](docs/SOFT_HEADING_VALIDATION.md).

Steering restart, terminal candidates and complete-route context changes, with one
full local round and unresolved gates: [liveness validation](docs/LIVENESS_VALIDATION.md).
