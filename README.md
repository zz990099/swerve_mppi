# Swerve MPPI Core

ROS-independent C++17 MPPI planning for a four-module chassis with dual Ackermann,
spin and crab modes. The controller samples velocity sequences within mode branches,
predicts transition costs, checks trajectories and returns body velocity with an
optional explicit mode request. It waits for measured mode confirmation before driving.

Version 0.24 completes observation, timing and fault preparation. Public source time
uses integer nanoseconds. Observation, computation, publication, application and
expiry remain distinct; bounded pairing, freshness and command-history evidence
replace exact timestamp equality. Runtime clock, feedback, motion, compute and
transition faults latch until a fresh observation passes `Controller::recover`.

The current Python chassis mechanics remain covered by 7,960 direct command-cycle
comparisons. Prediction can carry reconciled hidden command state when the caller
reports a precise prior application; missing or bounded-uncertain application stays
explicit and cold-seeded. This is not a completed ROS/Gazebo integration. See the
[complete staged plan](docs/PREPARATION_PLAN.md).

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
`SWERVE_MPPI_CHASSIS_SOURCE_DIR=/absolute/path/to/swerve_gazebo_sim` enables offline
parity against that checkout's actual Python mechanics. CI pins the reviewed companion
commit; ordinary library builds do not require it.
`BUILD_SHARED_LIBS=ON` selects a shared library. No ROS environment is required.

Consumers use the installed CMake package:

```cmake
find_package(swerve_mppi 0.24 CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE swerve_mppi::core)
```

## Use

Provide measured state, its source timestamp, a decision timestamp, an ordered path
and obstacles to `Controller::compute`.
A present command contains the requested mode, body-frame target velocity and an
optional mode request. A present zero velocity is an ordinary hold/brake intent.
An absent command provides no motion authorization; the caller must stop sending
motion commands and handle the chassis stop/fault policy. Record publication with
`set_publication_stamp`, enforce `command_valid_at`, and report the prior publication
and application window on the next input. Never reuse an old drive.

The core does not publish ROS messages, operate wheel joints, run a watchdog or
confirm the physical completion of a mode change. Stage 4 will translate this output
to the current chassis interface and exercise message sequences offline.

- [Architecture](ARCHITECTURE.md)
- [Command and observation contract](docs/CHASSIS_COMMAND.md)
- [Configuration](docs/CONFIGURATION.md)
- [Navigation](docs/NAVIGATION.md)
- [Model and feedback limits](docs/MODEL_FEEDBACK.md)
- [Validation](docs/VALIDATION.md), [performance](docs/PERFORMANCE.md), [development](docs/DEVELOPMENT.md)
