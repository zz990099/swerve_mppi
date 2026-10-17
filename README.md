# Swerve MPPI

A C++17 local velocity controller for a four-module chassis with dual Ackermann,
spin and crab modes. It samples control sequences, predicts nominal chassis
response, scores trajectories through injectable critics and returns a body
velocity plus an explicit target mode.

Version **0.28.0** is a breaking simplification. The core owns optimization and
mode selection. The ROS caller owns paths, transforms, costmap snapshots, chassis
message adaptation and goal completion. Ordinary velocity commands are computed
periodically without per-command acknowledgements. There are no execution-evidence
services, independent trajectory validators or task/recovery state machines in core.

## Packages

| Package | Responsibility |
| --- | --- |
| `swerve_mppi_core` | Plain CMake library; model, sampling, MPPI update and critic interface |
| `swerve_mppi_ros` | Jazzy node, external GoalChecker, ROS costmap critic and current chassis adapter |

## Standalone build

```bash
cmake -S swerve_mppi_core -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install-core"
```

```cpp
#include <swerve_mppi/controller.hpp>

swerve_mppi::Controller controller;
swerve_mppi::Input input;
input.path = {{0, 0, 0}, {1, 0, 0}};
input.goal = input.path.back();
// Fill measured pose, velocity, steering, wheel speeds and mode each cycle.
auto result = controller.compute(input);
// result.command.velocity: body-frame m/s, m/s, rad/s.
// result.command.mode: explicit desired mode, not a completion acknowledgement.
```

Consumers use `find_package(swerve_mppi_core 0.28 CONFIG REQUIRED)` and
`target_link_libraries(my_controller PRIVATE swerve_mppi::core)`.

## Gazebo path following

Use ROS 2 Jazzy / Gazebo Harmonic. Put this repository and
[`swerve_gazebo_sim`](https://github.com/zz990099/swerve_gazebo_sim) in a colcon
workspace. The reviewed companion revision is `9358c96884d7fa4866cea7c053daa7a7cec82d8f`.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to swerve_mppi_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
ros2 launch swerve_mppi_ros demo.launch.py
```

From another sourced terminal, wait for `/odom` and the controller subscription,
then publish one ordered path in the `odom` frame:

```bash
ros2 topic pub --once /path nav_msgs/msg/Path '{header: {frame_id: odom}, poses: [{header: {frame_id: odom}, pose: {orientation: {w: 1.0}}}, {header: {frame_id: odom}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}]}'
ros2 topic echo /mppi_controller/goal_reached
```

The default launch tracks in an obstacle-free scene with the costmap critic disabled.
The first-stage acceptance target is path tracking plus position/yaw convergence.
The nominal model includes actuator slew and stop/align switching; servo/contact/slip
calibration and obstacle navigation are separate validation work.

- [Architecture and API](ARCHITECTURE.md)
- [Configuration and ROS operation](docs/ROS.md)
- [Validation and remaining work](docs/VALIDATION.md)

Powered by Codex.
