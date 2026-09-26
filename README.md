# Swerve MPPI Core (prototype v0.1)

ROS-independent C++17 controller prototype for a four-wheel-steering base with
three discrete modes: dual Ackermann, spin in place, and crab. It predicts a
brake/align/confirm transition, samples continuous controls within separate
mode branches, and commits a real transition only after comparing its cost
with that of staying in the current mode.

See [ARCHITECTURE.md](ARCHITECTURE.md) for module responsibilities, the
feedback contract, and planned extension points.

## Build and test

In the prepared workspace, enable the locally installed CMake 3.28.3
and CTest (run this from the workspace root):

```bash
source ./activate-swerve-build.sh
cmake -S swerve_mppi -B swerve_mppi/build-cmake \
    -DSWERVE_MPPI_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build swerve_mppi/build-cmake -j 2
ctest --test-dir swerve_mppi/build-cmake --output-on-failure
```

To install the library and CMake package to a local prefix:

```bash
cmake --install swerve_mppi/build-cmake --prefix swerve_mppi/install-local
```

A downstream CMake target can then use
`find_package(swerve_mppi 0.1 CONFIG REQUIRED)` with
`CMAKE_PREFIX_PATH` pointing at `swerve_mppi/install-local`, and link against
`swerve_mppi::core`.

For a standalone copy with CMake already on `PATH`, run these commands from
the project directory:

```bash
cmake -S . -B build -DSWERVE_MPPI_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

GCC-only build (if CMake is unavailable):

```bash
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Iinclude \
    src/model.cpp src/optimizer.cpp src/controller.cpp tests/core_tests.cpp \
    -o build/swerve_mppi_tests
./build/swerve_mppi_tests
```

## Control contract

Call `Controller::compute(input)` with a timestamped measured vehicle state,
wheel feedback, a local path, and circle obstacles. The result contains a
mode-aware action, body and wheel commands, and diagnostics. `RequestMode` is
an idempotent request: the adapter must safely brake and align the wheels,
then report the new `actual_mode` **and** set `mode_confirmed` only when the
hardware has finished the transition. No moving command is produced while
the controller waits for that confirmation. Call `reset()` only after a
deliberate recovery from a latched fault; do not automatically clear one.

The timestamps must increase on every call. The vehicle's mode age is the
time since the **actual** mode was confirmed. `reference_path` must contain at
least one pose; its last pose is treated as the local goal. Coordinates and
units are metres, radians, and seconds in one consistent world frame. Body
commands and wheel targets use the robot frame. Wheel indices are front-left,
front-right, rear-left, rear-right.

## Implementation limits

- Obstacles are static circles and the robot footprint is a circle; the
  Nav2 costmap and full swept footprint are not implemented yet.
- The dual Ackermann mode uses a planar curvature bound and four wheel
  inverse kinematics. Actual linkage, steering stops, wheel acceleration,
  slipping, and actuator communication delays need calibration and further
  modeling for a real vehicle.
- Each sampled trajectory makes at most one mode change. Alignment and
  confirmation duration in rollouts are estimates; execution always waits
  for real hardware feedback, with a configurable timeout.
- This is a practical hybrid MPPI prototype. Gaussian sample weighting is
  applied within a fixed mode branch; mode selection compares branch costs.
  A formal importance-sampling derivation for the hybrid proposal is outside
  this prototype's scope.
- The optimizer uses single-threaded CPU sampling. Control rate and rollout
  counts must be benchmarked on the intended computer before deployment.
