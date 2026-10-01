# Stage 1 core behavior validation

Package version: 0.4.0. Comparison baseline: 16a5e54ca39ccf245e8f0f6458a904b9e025d76d.
No ROS or Gazebo integration was performed.

All builds use CMake 4.4.3 with GNU 13.3.0 and C++17, with warnings treated as
errors. CTest includes core, model, optimizer and execution suites, four
multi-seed behavior scenarios, and an installed downstream CMake consumer.

| Configuration | Result |
| --- | --- |
| Release static library | 9/9 CTest entries passed |
| Debug static library | 9/9 CTest entries passed |
| AddressSanitizer and UndefinedBehaviorSanitizer | 4/4 core CTest entries passed |
| Formatting and whitespace | clang-format and git diff --check passed |

The sanitizer command excludes installed_consumer and behavior_*; the twenty
behavior runs are covered by Debug and Release. ASAN_OPTIONS=detect_leaks=0 is
used, so no leak-detection result is claimed. No new shared-library validation
is claimed in this stage.

## Default-noise closed loops

Each scenario starts stopped in DualAckermann with a confirmed mode age of two
seconds. All Config defaults are retained except random_seed, which takes values
1, 7, 42, 73 and 101. Each run lasts 120 ticks at 0.1 seconds. The independent
fixture limits encoder acceleration and steering rate, computes body twist from
module velocity vectors, and integrates pose at the midpoint heading. It does
not advance the plant through DriveModel, TransitionModel, Kinematics::forward
or any predicted controller state. It allows bounded steering while driving.

| Scenario | Target | Maximum final position error over 5 seeds | Maximum final yaw error | Longest unfinished stationary interval | Confirmed mode changes |
| --- | --- | --- | --- | --- | --- |
| Straight | (1, 0, 0) | 0.028 m | 0.059 rad | 0.0 s | 0 |
| Lateral | (0, 1.4, 0) | 0.100 m | <0.001 rad | 1.2 s | 1 |
| Curve | 2 m radius arc through 0.7 rad | 0.037 m | 0.191 rad | 0.1 s | 0 |
| Spin | (0, 0, 1.8) | <0.001 m | 0.098 rad | 0.6 s | 1 |

Acceptance requires final position error below 0.2 m, final yaw error below
0.35 rad, no execution fault, and no unfinished stationary interval of three
seconds. A task is considered unfinished for the stall metric when position
error exceeds 0.3 m (or yaw error exceeds 0.3 rad for Spin). Stationary means
both linear and angular speed are below 0.02 in their respective SI units.
Straight/Curve permit no mode changes; Lateral/Spin permit at most one. The
fixture also checks joint travel and wheel-speed bounds. The model suite checks
coupled steering/wheel acceleration over smoothly varying curvature.

Running the same Lateral fixture against the baseline with seed 42 fails:
final position error 1.125 m, 103 Hold ticks, and a 10.3 s unfinished stationary
interval. Stage 1 produces 0.0444 m error, 44 Hold ticks and a 1.2 s maximum
interval for that seed. Hold counts include behavior near the goal; they are
not themselves treated as evidence of failure.

These are deterministic standalone research regressions, not a statistical
success-rate estimate or a physical tracking benchmark. Near-goal jitter,
terminal stopping, goal completion, arbitrary path progress/pruning and
calibrated actuator dynamics remain outside stage 1 acceptance.

## New boundary and model checks

- NaN and positive/negative infinity in all three body command components must
  latch a fault without nonzero wheel targets.
- Continuous curvature changes must remain in Drive while satisfying steering,
  wheel and full body acceleration/deceleration bounds.
- A mechanical hard-stop crossing still brakes before realignment.
- Moving steering targets beyond angle or per-tick rate limits are rejected.
- Same-mode entry intent survives path changes during alignment, times out for
  a stalled actuator, and is cancelled by reset or newly detected collision.
- Rollout and execution both retain alignment intent; ignored control ticks are
  masked, and a scheduled mode switch cannot preempt a committed alignment.
- Correlated-noise weighting checks the temporal precision operation, masking,
  nonzero-noise reset reproducibility and invalid correlation configuration.
- Confirmed mode entry preserves its intent through the first Drive.

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

CI runs the Debug/Release core, multi-seed behavior and installed-consumer checks
on Ubuntu 24.04.
These results establish standalone core and execution protocol behavior. The
encoder fixture is not a calibrated dynamics model; physical/Gazebo tracking,
slip, actuator-delay identification, command watchdogs, transport timing and ROS
adapters remain outside this validation scope.
