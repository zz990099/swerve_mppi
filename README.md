# Swerve MPPI Core

ROS-independent C++17 MPPI planning for a four-module chassis with dual Ackermann,
spin and crab modes. The controller samples velocity sequences within mode branches,
predicts transition costs, checks trajectories and returns body velocity with an
optional explicit mode request. It waits for measured mode confirmation before driving.

Version 0.22 completes stage 1 of the chassis-interface refactor. The production
library owns planning and prediction. Chassis execution, joint-profile sampling,
transport sessions and old Gazebo migration tools have been removed. There are no
compatibility wrappers for the removed interfaces.

The prediction model is still the nominal core model; matching the current Python
Gazebo chassis is stage 2. This release is not a completed Gazebo integration.
See the [complete staged plan](docs/PREPARATION_PLAN.md).

## Build and test

Requirements: CMake 3.20+, a C++17 compiler and a CMake-supported build tool.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install-local"
```

Use `SWERVE_MPPI_BUILD_TESTS=OFF` for a library-only build or
`SWERVE_MPPI_BUILD_BENCHMARKS=ON` for offline measurement tools.
`BUILD_SHARED_LIBS=ON` selects a shared library. No ROS environment is required.

Consumers use the installed CMake package:

```cmake
find_package(swerve_mppi 0.22 CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE swerve_mppi::core)
```

## Use

Provide measured state, an ordered path and obstacles to `Controller::compute`.
A present command contains the requested mode, body-frame target velocity and an
optional mode request. A present zero velocity is an ordinary hold/brake intent.
An absent command provides no motion authorization; the caller must stop sending
motion commands and handle the chassis stop/fault policy. Never reuse an old drive.

The core does not publish ROS messages, operate wheel joints, run a watchdog or
confirm the physical completion of a mode change. The future adapter will translate
this output to the current chassis interface and enforce message freshness.

- [Architecture](ARCHITECTURE.md)
- [Command and observation contract](docs/CHASSIS_COMMAND.md)
- [Configuration](docs/CONFIGURATION.md)
- [Navigation](docs/NAVIGATION.md)
- [Model and feedback limits](docs/MODEL_FEEDBACK.md)
- [Validation](docs/VALIDATION.md), [performance](docs/PERFORMANCE.md), [development](docs/DEVELOPMENT.md)
