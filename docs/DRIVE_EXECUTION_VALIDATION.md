# Drive actuator agreement and bounded recovery (0.11.0)

Review baseline: bcd3fd7c2fe65a9b51b53a89d58e0af95169afe1 (0.10.0).
Measured core and benchmark source: e504a4abade979ec3160a3723c5c0f53d6d73774.
Recorded on 2026-10-01. Scope remains the standalone core and reference executor.

## Drive interpolation and stopping displacement

Drive now reaches the endpoint wheel-speed and steering arrays by affine
interpolation over the whole control period. The previous prediction could reach
a deceleration endpoint early and hold it, while the independent actuator fixture
interpolated throughout the tick. For measured speed 0.1 m/s, a first Drive ending
at 0.048 m/s over 0.1 s, followed by a 1 m/s^2 Brake, the complete displacement is
0.008552 m. The previous 0.007304 m prediction passed an obstacle boundary at
0.008 m. The corrected continuation rejects it; the controller can select a
lower, freshly validated capture intent and the independent Drive/Brake fixture
stays outside that boundary.

Fixed steering produces an affine body twist. Straight/fixed-curvature motion
integrates exactly in SE(2); other fixed-angle ramps retain bounded quadrature.
Moving steering integrates each affine encoder vector analytically for yaw and
uses eight midpoint intervals for translation. If A bounds world acceleration,
translation error is at most A*dt^2/32 and curve-to-chord deviation is at most
A*dt^2/8. Per-segment margins include integration error and accumulated position
error. The angular velocity bound includes its possible interior departure from
the endpoint chord, using the second derivative of the encoder vector field.

Drive interpolation is also checked for pointwise body and joint rates. For fixed
steering, the minimum-speed fraction determines which acceleration/deceleration
budgets apply; a reversal uses the smaller applicable rate over the uniform tick.
Moving steering evaluates the exact summed body derivatives at fractions 0, 0.5
and 1. Every unsampled time is within dt/4 of a derivative sample; an analytic
second-derivative bound encloses the unsampled rates. Keeping signed vector sums
preserves cancellation between modules, instead of charging translation against
an artificial angular-acceleration sum.

Brake retains measured steering and proportionally scales the measured wheel
speeds toward zero at the fastest common body/joint braking rate. It holds zero
for any remaining time. These Drive and Brake profiles are explicit adapter
contracts. Calibrated actuator agreement is required; unspecified servos and tire
slip are outside the modeled enclosures.

## Execution boundaries and recovery

The reference executor checks absolute DualAckermann vx/yaw-rate, Crab translation
speed and Spin yaw-rate caps separately from the transient mode-projection
allowance. Exact kinematic targets cannot use that allowance to exceed a configured
speed cap. Endpoint consistency, module residuals, mechanical steering limits and
pointwise interpolated rates remain separate checks. Measured overspeed recovers
through the separately validated Brake path rather than issuing an overspeed Drive.

When a preferred first Drive plus complete stopping continuation is rejected, the
controller retries amplitude halvings of vx, vy and wz, with the full validator
on every attempt. safety_reduction_attempts defaults to 8, is limited to 16, and
zero disables retries. The budget applies per candidate; Output::safety_reductions
counts reduced validations across the entire compute call. No monotonic-safety
assumption is made about reducing speed. Exhaustion falls back to the separately
checked current stop. An immediate switch candidate can also fall back to its
finite keep-mode solution. Direction/curvature and frozen entry geometry remain
unchanged; alignment retains its original deadline. A successful reduction clears
the optimizer warm start, so an unvalidated preferred sequence is not reused.

A low-deceleration reproduction (0.001 m/s^2, 200 stopping ticks) previously waited
indefinitely for its preferred command. The bounded retry now finds an executable
slow intent with positive independently measured progress. Disabling retries
retains checked waiting behavior. No-feasible-plan and unsafe-current-stop semantics
remain distinct.

Ackermann proposals also anticipate the local ordered path's geometric curvature
and yaw-rate budget. The resulting translation cap is applied to nominal, sampled
and weighted controls, preserving each control's curvature. Crab and Spin retain
their own limits. Default lookahead, critic weights and behavior acceptance
thresholds were retained.

## Regression evidence

GNU 13.3.0, CMake 4.4.3, C++17, x86_64 Linux; -Werror:

| Check | Result |
| --- | --- |
| Release, benchmark enabled | 22/22 CTest entries passed |
| Debug, benchmark enabled | 22/22 CTest entries passed |
| AddressSanitizer + UndefinedBehaviorSanitizer | Seven core suites passed |
| Behavior matrix | Thirteen scenarios, five seeds each in Debug and Release |
| Installed consumer | Finds 0.11 and exercises the public continuation/sweep interfaces |
| Allocation smoke | Existing straight/seed-42 peak-at-most-200 gate passed |
| Formatting/whitespace | clang-format and git diff --check passed |

Independent moving-steering oracles use 20,000 substeps at periods 0.05, 0.1 and
0.2 s, without production kinematics, motion-profile or yaw-integration helpers.
They compare pose/error bounds, intermediate sweep enclosure and finite-difference
body rates. The complete-Brake analytic matrix remains intact. Executor tests
exercise exact speed caps and two excesses for all three modes, signed reversal
rates, residual limits and the existing 720-case transition matrix. An observing
critic checks the radius-0.5 curve speed bound across all optimized proposals and
verifies that Crab retains its translation budget.

Safety regressions cover the 8 mm obstacle reproduction, low-deceleration progress,
retry disablement/configuration bounds and healthy measured-overspeed braking.
Shared injected constraints still cover tracking, terminal capture and alignment.
The rejection fixture uses a smaller translation boundary because full-tick
pointwise-limited steering now produces smaller valid initial steps.

Sanitizers use -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie and
-no-pie, with ASAN_OPTIONS=detect_leaks=0. Behavior, installation and benchmarks are
excluded from sanitizer runs; no leak-detection claim is made. One local Release
executable lacked execute permission; restoring it and rerunning that suite passed.

## Current Release measurement

The benchmarks ran serially after all builds/tests, without CPU pinning, on a
shared x86_64 host reporting AMD EPYC 9V74. Defaults use a 0.1 s tick, 20-step
horizon, 80 samples per branch and two iterations. Compute timing includes the
cold first call; constructor/setup, fixture actuation and CSV output are excluded.
The matrix has 65 core runs and five runs each for a 401-point dense arc and an arc
with forty distant circles: 75 successful completions. Seeds are 1, 7, 42, 73, 101.

The table reports completion ranges, the maximum per-run planning P95 and peak
compute allocation count over those five seeds:

| Scenario | Completion (s) | Planning P95 (ms) | Allocation peak |
| --- | --- | --- | --- |
| straight | 3.5–3.8 | 37.311 | 113 |
| lateral | 7.3–14.7 | 28.183 | 117 |
| curve | 6.4–7.1 | 35.105 | 117 |
| spin | 4.3 | 0.000 | 16 |
| reverse | 3.5–3.8 | 30.011 | 113 |
| final_yaw | 6.7–7.6 | 30.757 | 113 |
| scurve | 12.4–12.6 | 34.840 | 118 |
| cusp | 6.7–7.2 | 31.488 | 113 |
| loop | 20.5–26.0 | 30.218 | 113 |
| short_cusp | 2.5 | 0.000 | 13 |
| short_loop | 4.6 | 0.000 | 13 |
| short_corner | 2.9 | 0.000 | 13 |
| near_obstacles | 8.2–16.8 | 45.653 | 122 |
| dense_curve | 6.3–6.9 | 43.650 | 121 |
| obstacles | 6.4–7.1 | 53.368 | 118 |

The maximum observed compute time was 95.682 ms, with zero 100 ms period overruns.
The largest allocation peak was 122. All 75 rows have zero Waiting calls and zero
safety-reduction validations: the dedicated regressions establish retry behavior,
while this matrix measures the default workload. These observations do not bound
worst-case retry cost or establish a hard real-time guarantee on a shared host.

Near-obstacle runs completed in 8.2–16.8 s with minimum measured per-tick segment
clearance 0.094667 m after circle/radius/margin inflation. The model independently
validates its swept enclosure. The fixture's reported chord clearance is a
measurement diagnostic, not a certificate for arbitrary physical inter-tick motion.
Peak S-curve cross-track error across all seeds was 0.281948 m, within the unchanged
0.3 m behavior threshold. Historical 0.8 timing/completion records remain unchanged.

- [Core matrix, 65 runs](benchmarks/core_v0.11_release.csv)
- [Dense arc, five runs](benchmarks/dense_v0.11_release.csv)
- [Forty distant obstacles, five runs](benchmarks/distant_obstacles_v0.11_release.csv)

The CSV adds safety_reductions, separate from optimizer evaluated_rollouts and
fallback_updates. Reproduce with the Release commands in PERFORMANCE.md; omit a
seed argument to run all five seeds for a named scenario.

## Consumer migration

Rebuild consumers with find_package(swerve_mppi 0.11 CONFIG REQUIRED). Config and
Output layouts changed. Output::body_command is the endpoint twist; actuator
adapters must interpolate both Drive joint arrays over the full tick and implement
the declared proportional Brake profile. ROS, simulator and hardware integration
remain subsequent work, including actuator calibration and measured timing.
