# Core execution validation

Date: 2026-09-30. Baseline: bf0776184adb71b002d5bc2b45ae20ece6ddc7c5.
Package version: 0.3.0. No ROS or Gazebo integration was performed.

All builds use CMake. Local tools were CMake/CTest 4.4.3 and a GNU 13.3.0 C++17
toolchain selected by CMake. Builds enabled warnings as errors; Release, shared
and sanitizer builds additionally enabled Wall, Wextra and Wpedantic globally.

| Configuration | Result |
| --- | --- |
| Debug static library | 5/5 CTest entries passed |
| Release static library | 5/5 CTest entries passed |
| Release shared library | 5/5 CTest entries passed |
| AddressSanitizer and UndefinedBehaviorSanitizer | 4/4 core CTest entries passed |
| Formatting and whitespace | clang-format check and git diff --check passed |

The five CTest entries are core behavior, model regressions, optimizer
regressions, execution regressions and an installed downstream CMake consumer.
The four core suites contain 28 named regression scenarios, including a loop
covering all six directed mode changes. The installation check uses a fresh
prefix and consumer build tree, resolves find_package(swerve_mppi 0.3 CONFIG
REQUIRED), then links and executes both Controller and ModeExecutor APIs.

The sanitizer run uses ASAN_OPTIONS=detect_leaks=0. LeakSanitizer is disabled
because of runtime process-inspection restrictions; no leak-detection result is
claimed. Address and undefined behavior instrumentation remain enabled. The
installed consumer is validated separately in static/shared nonsanitized builds.

## Added execution checks

- All six DualAckermann/Spin/Crab transitions with a separate rate-limited
  encoder fixture, braking before steering and zero drive throughout handover.
- Frozen request ID/angles across retries and no second Crab alignment when
  driving in the committed entry direction.
- Rejection of stale acknowledgement IDs, premature steering confirmation and
  moving-wheel confirmation despite stopped odometry.
- Idempotent completion without resetting mode age; mutated request payloads,
  incompatible entry geometry, active-request replacement and nonfinite data.
- Original deadlines survive retries; late alignment cannot clear a fault;
  predictive transitions exceeding the execution deadline are infeasible.
- SafeStop cancellation, explicit stopped recovery, preserved ID high-water
  marks, replay rejection after reset and no integer wraparound.
- Persistent actual mode on zero drive; masking drive during a pending switch;
  wheel/body command consistency and backward-clock fault latching.
- Startup mode age preceding the clock origin and bounded measured-steering
  error without spurious rejection of valid drive commands.
- Controller/ModeExecutor lateral closed loop and stable curved Ackermann/Spin
  progress, with no DriveModel or TransitionModel stepping inside the fixture.
- Frozen entry intent in Gaussian proposals, so masked transition noise cannot
  secretly change the requested geometry.

Existing regressions continue to cover bounded signed inverse/forward
kinematics, wheel/body rate limits, temporary configuration ownership, branch
hysteresis, full pose horizons, swept circular collisions, effective projected
noise, reproducible reset, critic extension and invalid controller inputs.

CI runs the Debug/Release core and installed-consumer checks on Ubuntu 24.04.
These results establish standalone core and execution protocol behavior. The
encoder fixture is not a calibrated dynamics model; physical/Gazebo tracking,
slip, actuator-delay identification, command watchdogs, transport timing and ROS
adapters remain outside this validation scope.
