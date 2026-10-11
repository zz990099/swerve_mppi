# Swerve MPPI

ROS-independent C++17 MPPI planning for a four-module chassis with dual Ackermann,
spin and crab modes. The controller samples velocity sequences within mode branches,
predicts transition costs, checks trajectories and returns body velocity with an
optional explicit mode request. It waits for measured mode confirmation before driving.

Version 0.27 adds `swerve_mppi_ros/`: a Jazzy lifecycle node consuming the installed
public interface of `swerve_mppi_core/`. The unchanged 0.26 core remains plain
CMake with no ROS dependencies. Repository-wide documentation and developer tools
stay at the root. Task handling and transport belong to the ROS package.

The current-chassis adapter maps state plus odometry, converts wheel `rad/s` to
rolling `m/s`, preserves frozen mode receipts, rejects stale output and validates
shared chassis parameters. Publication/application evidence stays explicit because
the state message does not echo an MPPI command ID.

The current Python chassis mechanics remain covered by 7,960 direct command-cycle
comparisons. Prediction can carry reconciled hidden command state when the caller
reports a precise prior application; missing or bounded-uncertain application stays
explicit and cold-seeded. ROS transport is implemented; physical execution evidence,
calibration and closed-loop Gazebo acceptance remain stage 5C. See the
[complete staged plan](docs/PREPARATION_PLAN.md).

## Build and test

Requirements: CMake 3.20+, a C++17 compiler and a CMake-supported build tool.

```bash
cmake -S swerve_mppi_core -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install-local"
```

Use `SWERVE_MPPI_BUILD_TESTS=OFF` for a library-only build or
`SWERVE_MPPI_BUILD_BENCHMARKS=ON` for offline measurement tools.
`SWERVE_MPPI_CHASSIS_SOURCE_DIR=/absolute/path/to/swerve_gazebo_sim` enables offline
parity against that checkout's actual Python mechanics. CI pins the reviewed companion
commit; ordinary library builds do not require it.
`BUILD_SHARED_LIBS=ON` selects a shared library. No ROS environment is required.

Consumers use the installed CMake package:

```cmake
find_package(swerve_mppi_core 0.26 CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE swerve_mppi::core)
```

With colcon installed, the same core can be built from the repository root without
sourcing ROS:

```bash
colcon build --packages-select swerve_mppi_core --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select swerve_mppi_core --return-code-on-test-failure
colcon test-result --all --verbose
source install/local_setup.bash
```

Use separate build/install directories when switching between CMake and colcon.
The repository root has no CMake project. The former `find_package(swerve_mppi)`
interface is removed; update consumers to the current package name.

## Use

Provide measured state, its source timestamp, a decision timestamp, an ordered path
and obstacles to `Controller::compute`.
A present command contains the requested mode, body-frame target velocity and an
optional mode request. A present zero velocity is an ordinary hold/brake intent.
An absent command provides no motion authorization; the caller must stop sending
motion commands and handle the chassis stop/fault policy. Generic integrations record
publication with `set_publication_stamp`, enforce `command_valid_at`, and report the
prior publication and application window on the next input. Current-chassis integrations
instead use the adapter's two-step `make_command` and `record_application` contract.
Never reuse an old drive.

The core does not publish ROS messages, operate wheel joints or run a watchdog.
`current_chassis::Adapter` supplies transport-free current-message DTOs, validation
and mapping; `swerve_mppi_ros` performs the mechanical ROS copy and publication gating.
See [ROS integration and lifecycle operation](docs/ROS_INTEGRATION.md) for Jazzy
build commands, task inputs, application evidence and recovery.

- [Architecture](ARCHITECTURE.md)
- [Command and observation contract](docs/CHASSIS_COMMAND.md)
- [Current chassis offline adapter](docs/CURRENT_CHASSIS_ADAPTER.md)
- [ROS integration](docs/ROS_INTEGRATION.md)
- [Configuration](docs/CONFIGURATION.md)
- [Navigation](docs/NAVIGATION.md)
- [Model and feedback limits](docs/MODEL_FEEDBACK.md)
- [Validation](docs/VALIDATION.md), [performance](docs/PERFORMANCE.md), [development](docs/DEVELOPMENT.md)
