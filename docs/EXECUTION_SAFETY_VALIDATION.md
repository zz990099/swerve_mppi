# Execution contract stage 1 (0.14)

Baseline: `4f1e6bbb4c09808002870b19047775539e1ccb89` (0.13.0).
Scope: standalone core execution-time validation, actuator reference, contracts
and regressions. No ROS/Gazebo adapter or physical calibration is included.

## Changes

- Command envelopes require the planning observation timestamp, scheduled application
  and execution-start expiry. Republishing cannot disguise an old observation.
- Review correction: envelopes also require an owned CommandTask snapshot of the
  originating path ID, heading policy and complete ordered path geometry. Mismatch
  or missing metadata rejects the queued Output before any mode-state preview, using
  checked braking or latching SafeStop if a safe stop cannot be established.
- TimedExecutor takes current ControllerInput and checks the exact execution-start
  state, actual joint interval and full stopping continuation against current
  circular obstacles and injected hard constraints.
- Supervisor previews are transactional. A rejected new mode request consumes no
  request ID and commits no unexecuted transition. A valid complete braking fallback
  is recoverable; an unsafe current stop latches SafeStop.
- ActuationModel returns a checked ActuationPlan with high-rate samples. Drive ramps
  both joint arrays over the full tick. Brake preserves steering and wheel-speed
  proportions; Hold/RequestMode finish complete braking before stationary alignment.
  SafeStop has no certified normal-braking profile.
- Supervisor and actuator reference share joint-target eligibility checks. The
  independent encoder behavior fixture remains a separate test oracle.

## Focused regressions

The delayed-command reproduction starts at 0.8 m/s with an obstacle at x=1.01 m
(radius 0.05 m, robot radius 0.5 m and margin 0.05 m). At x=0, one Drive tick plus
complete stopping ends at x=0.40 m and passes. At x=0.08 m one tick later, the
same still-recent command would stop at x=0.48 m and collide. The guarded executor
rejects that Drive, independently validates braking to x=0.40 m, and remains healthy.
A subsequent safe command recovers without reset.

Other regressions cover new obstacles, unsafe-stop latching, stopping-budget
exhaustion, injected constraints, transactional request IDs, incompatible validators
and actuator-model parameters, missing/nonfinite/source-expired/scheduled/expired
metadata, and recent raw feedback that is not aligned to the application instant.

Task regressions cover identical geometry with a new ID, reversed goals with the same
ID, interior x/y/yaw changes, goal yaw, heading policy, waypoint deletion/reordering,
missing snapshots, recovery on a freshly bound command and unsafe-stop failure.
Every stale-task Drive remains mechanically and collision valid, isolating the task
identity defect. Captured paths own their geometry; mutating the source cannot rebind
the queued command. A rejected obsolete mode request leaves ID 7 available for a
fresh request bound to the latest task.

An independent 4096-substep encoder integration checks actuator samples in all three
modes. Intermediate physical curves stay within the certified swept enclosure;
end poses agree within the reported integration bound. Brake tests check proportional
wheel speeds and retained steering. Residual 0.004 m/s motion consumes 0.004 s of
braking before Hold can align; its nonzero displacement is retained in validation.
Out-of-interval samples cannot extrapolate a previous Drive.

## Local verification

GNU 13.3.0, C++17:

| Check | Result |
| --- | --- |
| Optimized core, `-Wall -Wextra -Wpedantic -Werror` | Compiled |
| Optimized core suites | 8/8 passed |
| Direct behavior, 14 scenarios × 5 seeds | 70/70 passed |
| Guarded behavior, same matrix and independent plant | 70/70 passed without safety fallback |
| Debug core suites | 8/8 passed |
| AddressSanitizer + UndefinedBehaviorSanitizer core suites | 8/8 passed |
| Standalone consumer using the new public API | Compiled, linked and passed |
| Whitespace | `git diff --check` passed |

The local environment has no CMake executable or ROS/Gazebo runtime. Local builds
use the same source list through GNU compilation; the installed CMake consumer is
covered by the repository's Debug/Release CI, not claimed as a local install test.
CMake now registers direct and guarded versions of all fourteen behavior cases,
plus actuation regressions and the existing install/allocation checks.

Local sanitizers use non-PIE binaries and `detect_leaks=0`. LeakSanitizer cannot
inspect this environment's `/proc` tasks; no leak-detection result is claimed.
These tests establish the nominal execution contract, not agreement with an
uncalibrated plant, tire slip or real-time transport uncertainty.

## Migration and remaining work

Rebuild against `find_package(swerve_mppi 0.14 CONFIG REQUIRED)`. Replace state-only
TimedExecutor calls with current ControllerInput, provide all envelope timestamps,
include CommandTask::capture(planning_input) from the same full task that produced
the Output, and consume the returned ActuationPlan at the actuator rate. Do not
reconstruct task binding from later feedback. State time alignment
must be real; changing an old timestamp does not compensate motion. Pass the shared
planning validator if extra hard constraints are used.

Execution rejections must be reported from TimedExecutionResult; do not announce
successful execution/completion from an obsolete planner Output after a rejection.
The adapter owns exclusive joint-command publication, metadata, coherent current
constraints, time alignment and the independent actuator watchdog.

The configured decelerations specify a nominal commanded curve, not a minimum
guaranteed physical capability. Braking/steering response, latency and slip still
require independent simulation calibration. Latency prediction, uncertainty
envelopes, solver deadlines and tighter tracking objectives remain later work.
