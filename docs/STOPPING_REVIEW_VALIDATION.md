# Drive continuation, motion and time boundaries (0.10.0)

Review baseline: 40e81fda70e308363ef78bb132f01742ba40c447 (0.9.0).
Recorded on 2026-10-01. Scope remains the standalone C++ core; no ROS, simulator
or hardware integration was added.

## Common first-Drive and stopping gate

Controller::apply_control validates each selected first Drive and its complete
braking continuation before publishing Drive. Tracking, capture and committed
same-mode/post-switch alignment share that gate. Immediate switch requests also
check the committed entry, first Drive and stop before beginning the handshake.
Warm-start acceptance happens only after this validation. A rejected candidate
clears local/warm intent and falls back to a separately validated current stop;
only that valid stop can yield recoverable NoFeasiblePlan with Brake/Hold.
Rejected or incomplete current stopping emits UnsafeStoppingTrajectory/SafeStop.

RolloutEngine::generate_continuation retains the entry intent until alignment and
first Drive finish, then requests zero motion through complete modeled rest.
Config::stopping_horizon_steps (default 200 ticks) bounds this work independently
of the MPPI horizon. generate_stop likewise ends at zero modeled wheels/body,
including residual motion below the execution handover thresholds. Budget
exhaustion returns an invalid trace. A stationary stop contains only its initial
pose; controls and sweep margins correspond to actual consumed ticks. Stops
retain measured steering, mode, request ID and confirmation state.

Regressions reproduce a two-tick optimization horizon with 0.1 m/s^2 braking:
Tracking and Capture both reject a first command whose stopping tail collides,
but both allow Drive when the obstacle clears even though stopping exceeds that
optimization horizon. A two-tick stopping budget rejects the same Drive; an
incomplete current stop faults. These outputs are checked through ModeExecutor.
Existing injected constraints, terminal stops and unconfirmed handshakes remain
covered by the safety suite.

A zero control intent now emits Brake/Hold with measured steering and zero
wheel/body targets. It does not declare residual measured motion to be Drive.
The short-lookahead, 0.1 m/s^2 wheel-rate reproduction with speeds
{0.30, 0.24, 0.24, 0.30} therefore executes healthy braking even though its module
residual exceeds the Drive target allowance. Nonzero Drive checks remain intact.

## Displacement and swept motion

DriveModel retains encoder-derived endpoint joint/body agreement, but integrates
motion over a piecewise linear body-twist ramp. The linear speed minimum and
angular zero crossing split braking from acceleration. Each phase consumes the
maximum of its body and joint rate requirements. The model reduces the endpoint
joint fraction if that combined profile cannot fit the tick. If it reaches its
endpoint before the tick ends, remaining time holds the endpoint twist; braking
holds zero after reaching zero.

Commuting twists, including straight ramps and fixed-curvature braking, integrate
exactly using the time-average SE(2) exponential. Other phases use eight midpoint
quadrature intervals with exact ramp yaw. For a phase of duration T and bounded
world velocity derivative A, accumulated position error is at most A*T^2/(4*N).
A tick's curve-to-chord deviation is bounded by A*dt^2/8; its sweep margin includes
that radius, local quadrature error and previously accumulated position error.
Straight monotonic ramps need no curvature inflation. Signed reversals retain a
nonzero sweep enclosure even when their endpoints hide the interior excursion.
TrajectoryValidator checks these segment enclosures against inflated circles and
rejects malformed margins. Nonzero short segments include their far endpoint.

These bounds enclose the declared predictive body ramp. They do not certify
intermediate tire motion, slip or actuator response. Measured twist/encoders must
still be mutually consistent as required by the adapter contract. Transition
prediction brakes to zero before freezing its pose; measured execution continues
to use configured stopped tolerances and can therefore hand over earlier.

Independent checks include:

- 54 complete-stop analytic cases: both signs, three speeds (including
  sub-threshold motion), three periods and Ackermann/Crab/Spin. Expected linear
  displacement or yaw is initial speed times stopping duration divided by two;
  duration is set by the slowest body or rolling-speed limit.
- The 0.1 m/s braking reproduction now travels 0.005 m, matching v^2/(2*a).
  An obstacle with only 0.003 m initial clearance rejects the stop and yields
  SafeStop. This is a model regression, not a measured hardware collision claim.
- A curved stop is compared against an independent constant-curvature integral.
  A signed reversal checks braking, opposite acceleration and final-speed hold,
  and places an obstacle inside an excursion missed by its endpoint chord.
- Three noncommuting moving ramps are compared against a separate 20,000-substep
  integral. Their position error stays inside the published enclosure.
- The independent encoder behavior fixture now uses 64 substeps per tick,
  reconstructs intermediate twists from wheel vectors, includes rate-limited
  braking displacement and holds zero after braking completes. Its 0.005 m
  straight stopping result is checked independently of the production integrator.

## Inclusive time boundaries

Manager, executor, controller alignment, rollout/transition prediction, scheduler,
completion dwell and transport guards share numerical time comparisons. Inclusive
bounds use a 1 ns floor or four times double precision epsilon times the timestamp magnitude,
whichever is larger. Duration-to-ticks conversion applies the same tolerance to
avoid spurious extra ticks. Strict timestamp ordering and replay checks remain
strict. The two mandatory confirmation/handover cycles remain unchanged.

Regressions check exact and 10 microsecond-late receipt across four periods and
three clock scales, through both ModeManager and ModeExecutor (48 transactions).
They check the controller and rollout at a decimal same-mode alignment deadline,
and compare seven decimal alignment ticks plus two protocol cycles against the
measured handshake. Decimal confirmation allowances and scheduler dwell use the
same quantization. Existing 720 stopped-entry timing comparisons and timeout,
watchdog, stale-feedback and recovery tests continue to pass.

## Validation evidence

| Check | Result |
| --- | --- |
| Debug, -Werror, benchmarks enabled | 22/22 CTest entries passed |
| Release, -Werror, benchmarks enabled | 22/22 CTest entries passed |
| AddressSanitizer + UndefinedBehaviorSanitizer | Seven core suites passed |
| Default-noise behavior | Thirteen scenarios, five seeds each, both build types |
| Installed consumer | Finds 0.10 and exercises continuation/sweep APIs |
| Allocation smoke | Existing peak-at-most-200 gate passed in both builds |
| Formatting and whitespace | clang-format and git diff --check passed |

The sanitizer build uses -fno-pie/-no-pie and ASAN_OPTIONS=detect_leaks=0.
Behavior, installed-consumer and benchmark entries are excluded from that
seven-suite sanitizer run; no leak-detection claim is made. Existing behavior
completion, corridor and post-completion Hold criteria and seeds are unchanged.
No new wall-clock performance matrix was recorded; versioned historical CSVs
remain historical rather than measurements of 0.10.

Reproduction commands, using a C++17 compiler and CMake 3.20 or later:

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS=-Werror -DSWERVE_MPPI_BUILD_BENCHMARKS=ON
cmake --build build-release --parallel 4
ctest --test-dir build-release --output-on-failure --parallel 4
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS=-Werror -DSWERVE_MPPI_BUILD_BENCHMARKS=ON
cmake --build build-debug --parallel 4
ctest --test-dir build-debug --output-on-failure --parallel 4
cmake -S . -B build-sanitizers -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS=-Werror -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined -no-pie'
cmake --build build-sanitizers --parallel 4
ASAN_OPTIONS=detect_leaks=0 ctest --test-dir build-sanitizers --output-on-failure \
  -R '^(swerve_mppi_tests|model_regressions|optimizer_regressions|execution_regressions|navigation_regressions|safety_regressions|timing_regressions)$'
```

## Migration and remaining limits

Rebuild all downstream consumers with find_package(swerve_mppi 0.10 CONFIG REQUIRED).
Config, StepResult and Trajectory layouts changed; RolloutEngine adds the bounded
continuation API and TransitionModel accepts optional sweep/error outputs.
Consumers of variable stopping traces must no longer assume horizon_steps+1
poses. Empty caller-supplied sweep margins denote piecewise straight motion;
provided margins must be finite, nonnegative and match the segment count.

The stopping budget is a cap, not permission to truncate a stop. Increase it only
with an explicit computation budget and independent braking evidence. Validation
is deliberately conservative and can reject a feasible trajectory near an obstacle.
This core still uses circular static obstacles, at most one switch per optimization
horizon, seeded mode-entry geometry and no solver deadline. Simulator/actuator
calibration, physical braking/sweep comparison and timing measurement remain
necessary before claiming plant-level safety or tracking performance.
