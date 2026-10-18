# Swerve MPPI

A C++17 local velocity controller for a four-module chassis with dual Ackermann,
spin and crab modes. It samples control sequences, predicts nominal chassis
response, scores trajectories through injectable critics and returns body velocity
plus an explicit target mode.

The ROS package **0.29.0** implements a **Nav2 Jazzy controller plugin**, ordered
GoalChecker and chassis bridge. The standalone numerical core remains 0.28.0.
Nav2 owns FollowPath actions, control timing, TF, odometry, costmaps and completion.
The former standalone ROS `/path` controller is replaced by this integration.

| Package | Responsibility |
| --- | --- |
| `swerve_mppi_core` | Plain CMake model, sampling, MPPI update and critic interface |
| `swerve_mppi_ros` | Nav2 controller / goal checker plugins and final-velocity chassis bridge |

## Standalone core

```bash
cmake -S swerve_mppi_core -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

Consumers use `find_package(swerve_mppi_core 0.28 CONFIG REQUIRED)` and link
`swerve_mppi::core`. `Controller::compute(Input)` returns body-frame `vx, vy, wz`
and a target mode. Fill measured pose, velocity, steering and rolling wheel speeds
on each call. Standalone callers own ordered path preparation and goal progress.

## Nav2 / Gazebo test setup

Use ROS 2 **Jazzy** / Ubuntu 24.04 and Gazebo **Harmonic**. Put this repository and
[`swerve_gazebo_sim`](https://github.com/zz990099/swerve_gazebo_sim) in a colcon
workspace. The pinned companion is `9358c96884d7fa4866cea7c053daa7a7cec82d8f`.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to swerve_mppi_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
ros2 launch swerve_mppi_ros demo.launch.py
```

From another sourced terminal, send a path after the FollowPath server is active:

```bash
ros2 action send_goal /follow_path nav2_msgs/action/FollowPath "{path: {header: {frame_id: odom}, poses: [{pose: {orientation: {w: 1.0}}}, {pose: {position: {x: 1.0}, orientation: {w: 1.0}}}]}, controller_id: FollowPath, goal_checker_id: goal_checker, progress_checker_id: progress_checker}" --feedback
```

This launch supplies a FollowPath server with a free local costmap, plus the
companion chassis and bridge. It is ready for controller integration tests; a
full navigation system additionally needs localization, planning, sensors and BT
configuration. Obstacle and real-hardware acceptance are separate work.

- [Architecture](ARCHITECTURE.md)
- [Nav2 wiring, parameters and migration](docs/ROS.md)
- [Validation and remaining work](docs/VALIDATION.md)
