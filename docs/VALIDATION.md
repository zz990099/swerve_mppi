# Core refactor validation

Date: 2026-09-30. Baseline: c6a987ee6efdd7a8ed355bb7bf3c42a48c24443b.

All builds use CMake. Local tools were CMake/CTest 4.4.3 and a GNU 13.3.0 C++17
toolchain selected by CMake. Builds enabled warnings as errors.

| Configuration | Result |
| --- | --- |
| Debug static library | 4/4 CTest entries passed |
| Release static library | 4/4 CTest entries passed |
| Release shared library | Installed independent consumer passed |
| AddressSanitizer and UndefinedBehaviorSanitizer | 3/3 core CTest entries passed |
| Formatting and whitespace | clang-format check and git diff --check passed |

The four CTest entries are the original core behavior suite, model regressions,
optimizer regressions, and an installed downstream CMake consumer. The three
core suites exercise 16 behavioral scenarios. The installation check uses a
fresh prefix and consumer build tree, then resolves the exported target with
find_package(swerve_mppi 0.2 CONFIG REQUIRED), links it and executes it.

The sanitizer run disabled LeakSanitizer using ASAN_OPTIONS=detect_leaks=0.
The runtime cannot inspect process tasks under /proc, so enabling LeakSanitizer
causes an environment error before it can report a leak. Address and undefined
behavior instrumentation remained enabled. No leak-detection result is claimed.

The model regressions cover reachable equivalent steering directions, +/-90
degree limits, signed wheel-speed round trips, deceleration selection, matching
wheel/body velocities, wheel/body rate limits, braking before realignment,
zero-delay transition termination, measured wheel-stop gating and temporary
configuration ownership.

The optimizer regressions cover switch hysteresis through both the scheduler
and controller, full pose horizons and transition masks, swept obstacle
collisions, projected noise, disabled-noise dimensions, reproducible reset,
deterministic straight-path feedback progress, custom critic rejection and
invalid public inputs.

CI repeats the Debug/Release core and installed-consumer checks on Ubuntu 24.04.
This record does not assert the future remote CI outcome. No ROS/Gazebo combined
closed-loop test was performed in this stage; a typed persistent mode execution
contract and ROS adapter remain required.
