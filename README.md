# Swerve MPPI Core

A ROS-independent C++17 hybrid controller for a four-module chassis with dual
Ackermann, spin and crab modes. It evaluates continuous controls within fixed
mode branches, predicts braking and steering alignment, and waits for measured
mode confirmation before driving after a switch.

See [ARCHITECTURE.md](ARCHITECTURE.md) for component responsibilities, execution
contracts and remaining simulation integration work.

## Build, test and install

Requirements: CMake 3.20 or newer and a C++17 compiler. CMake is the only supported
build entry point; no workspace activation script or ROS environment is required.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install-local"
```

Set SWERVE_MPPI_BUILD_TESTS=OFF for a library-only build.
BUILD_SHARED_LIBS=ON selects a shared library on supported toolchains.
CTest runs the original behavior checks, model and optimizer regressions, and
an independent consumer that finds and links the installed CMake package.
CI builds Debug and Release configurations through CMake.

Downstream CMake projects use the exported target:

```cmake
find_package(swerve_mppi 0.2 CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE swerve_mppi::core)
```

Add the install prefix to CMAKE_PREFIX_PATH.

## Controller contract

Call Controller::compute(input) once per Config::dt_s using fresh, consistent
measured vehicle state and a nonempty local path. The current warm-start sequence
advances by one model step per accepted drive action; arbitrary controller/model
period ratios are not supported yet.

- Pose and path coordinates share one world frame. Velocities are in the robot
  frame. Lengths, angles and time use metres, radians and seconds.
- Wheel order is FL, FR, RL, RR. Core wheel speeds are **linear rolling speeds in
  m/s**. Convert joint rad/s using wheel_radius_m; do not copy raw JointState
  velocities into VehicleState::wheel_speeds.
- Steering angles are mechanical joint positions bounded by
  steering_limit_rad. Defaults use +/-90 degrees. Signed drive speeds provide
  equivalent wheel directions without crossing a steering stop.
- Timestamps must be finite, nonnegative and strictly increasing. The adapter
  owns feedback freshness checks, clock-reset handling and coordinate conversion.
- time_in_mode_s is the age of the actual confirmed mode, not the requested
  mode. mode_confirmed must come from the execution layer.
- RequestMode is an idempotent explicit request. The executor brakes, aligns
  and acknowledges it. A zero velocity message cannot encode this action.
- Brake requests controlled stopping while retaining steering.
  Hold keeps drive stopped while applying its steering targets.
  SafeStop disables drive; its numeric targets must not be interpreted as a
  mode change or a recovery request.
- After a latched mode fault, call reset() only following deliberate recovery.

## Current status

The default chassis dimensions, wheel radius, steering rate and wheel limits
match the bundled swerve_gazebo_sim defaults. Body speed/acceleration limits
remain conservative planning settings; they are not calibrated Gazebo dynamics.
Components own their configuration, so temporaries and copied controllers cannot
leave dangling configuration references.

Version 0.2 moves branch generation from Optimizer::make_branches() to
ModeScheduler::make_branches(). TransitionModel now takes only Config.
The public Controller::compute() interface remains available.

This is a core research prototype. It has no ROS node or Nav2 plugin, and has not
been validated in a combined Gazebo closed loop. Obstacles and the footprint are
circles, the local goal is the last path pose, and each horizon allows one mode
change. Path progress/pruning, goal completion, footprint/costmap queries,
actuator-delay calibration and a typed mode command/feedback interface are later
integration work.
