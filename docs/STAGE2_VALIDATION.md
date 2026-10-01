# Stage 2 navigation validation

Package version: 0.5.0. Implementation baseline: b6429f327d12c5bdee03a2ea4d96f2a27c056d87.
[Stage 1 results](STAGE1_VALIDATION.md) are retained as a historical report.
No ROS, Gazebo or hardware integration is claimed here.

GNU 13.3.0, CMake 4.4.3, C++17, warnings treated as errors:

| Configuration | Result |
| --- | --- |
| Release static library | 15/15 CTest entries passed |
| Debug static library | 15/15 CTest entries passed |
| AddressSanitizer + UndefinedBehaviorSanitizer | 5/5 core suites passed |
| Formatting/whitespace | clang-format and git diff --check passed |

CTest comprises five core suites, nine behavior scenarios and an installed CMake
consumer. Sanitizers exclude behavior and installed-consumer tests; they use
-fsanitize=address,undefined -fno-omit-frame-pointer with non-PIE executables and
ASAN_OPTIONS=detect_leaks=0. No leak-detection or new shared-library result is
claimed. CI runs the full Debug/Release matrix on Ubuntu 24.04.

## Completion regressions

All Config defaults are retained except random_seed in {1, 7, 42, 73, 101}. Each
run starts stopped in confirmed DualAckermann with two seconds of mode age. The
independent encoder fixture limits joint rates, sums module velocity vectors and
integrates pose at midpoint heading. It never advances the plant using DriveModel,
TransitionModel, Kinematics::forward or controller-predicted state.

All 45 runs must complete within 40 simulated seconds, satisfy measured body/wheel
stop and both 0.06 m / 0.05 rad pose tolerances, then remain in Hold for at least
one simulated second. Additional gates reject execution faults, path error >=0.3 m,
unfinished stationary intervals >=3 seconds and more than four mode changes.
Unfinished uses remaining arc length and actual goal XY/yaw, so a closed loop at
its start cannot evade the stationary metric. Stationary means linear/angular
speeds below 0.02 in SI units; this metric differs from the stricter wheel-stop
completion gate and from Output::stalled.

The table gives worst errors, longest stationary interval and maximum mode-change
count over five seeds; completion time is the range. Error values are rounded up.

| Scenario | Completion time (s) | Final XY error (m) | Final yaw error (rad) | Max path error (m) | Stationary interval (s) | Mode changes |
| --- | --- | --- | --- | --- | --- | --- |
| Straight 1 m | 3.8–3.9 | 0.055 | 0.026 | 0.011 | 0.0 | 0 |
| Lateral 1.4 m | 7.1–8.9 | 0.057 | 0.001 | 0.198 | 1.3 | 1 |
| Arc: R=2 m, 0.7 rad | 6.7–7.0 | 0.060 | 0.050 | 0.069 | 0.9 | 2 |
| Spin 1.8 rad | 4.4–4.4 | 0.001 | 0.043 | 0.001 | 0.6 | 1 |
| Reverse 1 m | 3.7–3.9 | 0.060 | 0.023 | 0.018 | 0.0 | 0 |
| 1 m + terminal yaw 1.2 rad | 7.4–7.8 | 0.060 | 0.046 | 0.029 | 0.8 | 2 |
| S-curve: 3 m, amplitude 0.35 m | 9.9–10.2 | 0.060 | 0.050 | 0.249 | 1.0 | 2 |
| Out-and-back 1 m reversal | 6.9–7.0 | 0.059 | 0.031 | 0.059 | 0.1 | 0 |
| Closed 1 m square | 17.6–25.1 | 0.055 | 0.040 | 0.199 | 1.3 | 1 |

The S-curve uses tangent body yaw. Lateral and square paths keep body yaw zero;
reverse/reversal keep yaw zero while changing translation sign. The terminal-yaw
scenario uses GoalOnly. The square and reversal end at their starting position,
so successful completion also checks ordered path traversal.

## Boundary regressions

- Bounded monotonic progress, local pruning, geometry/ID changes and no nearest
  branch jump at a crossing; closed paths cannot immediately complete.
- Duplicate positions, shortest-angle yaw interpolation, zero-length final yaw,
  and a duplicate-point reversal that truncates lookahead until capture.
- Body odometry cannot hide moving wheels; interrupted confirmation restarts the
  settling dwell; continued wheel motion triggers a stall diagnostic.
- Identical tasks restart by path_id; measured progress clears a stall.
- Replanning leaves active mode request ID/targets/deadline intact and clears
  obsolete same-mode alignment intent.
- Terminal speed caps constrain nominal, sampled and weighted controls, while
  mode-entry proposals preserve their frozen constrained seed.
- Existing model, noise, collision and all six execution transition checks remain
  in the suite. The installed consumer requests the 0.5 package.

## Limits and next work

These deterministic regressions establish task lifecycle and standalone completion,
not an arbitrary-path success guarantee. The S-curve reaches roughly 0.25 m peak
path error and the square takes up to 25.1 seconds; lookahead/cost tuning and
corner efficiency still need improvement. Tests do not model slip, calibrated
actuator delay or transport jitter. Avoidance still uses a circular footprint;
the near-goal policy can conservatively stop when its constant-intent horizon
intersects an obstacle.

The next stage should integrate the explicit mode handshake with the simulator,
compare predicted/actual steering and stopping response, and measure solve-time
percentiles alongside tracking error, task completion and mode changes. Add
costmap/footprint and Nav2 action lifecycle adapters after that contract is validated.
