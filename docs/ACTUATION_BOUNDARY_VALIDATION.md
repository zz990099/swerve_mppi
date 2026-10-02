# Actuation boundary review and validation (0.12.0)

Review baseline: 31a59ee3c47f4838ed68896771da62c1c42dc97f (0.11.0).
Measured core and benchmark source: e08a92ad4491c871c8d06a8301609a7a5c2114b1.
Recorded on 2026-10-02. Scope is the standalone core and reference executor.

## Residual rolling during alignment

The stopped thresholds authorize protocol handover, but do not mean zero rolling
speed. Previously a Hold could steer while braking subthreshold wheel speeds;
prediction retained a straight body ramp and extended braking to the steering
time. With dt=0.5 s, measured forward/rolling speed 0.2 m/s and stopped thresholds
0.25/0.21 m/s, the old prediction ended at (0.05, 0). Independently integrated
simultaneous braking/steering ended near (0.0195822, 0.00329208). An obstacle using
the default footprint and margin could pass prediction but overlap that measured
footprint by 0.2 mm.

The 0.12 contract orders actuation: finish proportional braking with retained
measured steering, then align using only the remaining stopped part of the tick.
Brake always retains measured steering. Hold/RequestMode may supply an alignment
target after the stopped-threshold gate, but the actuator must not rotate rolling
wheels. Both local alignment and explicit transition prediction use the ordered
profile. Confirmation ticks also retain any residual braking displacement rather
than silently setting wheel/body speeds to zero.

In the reproduction, braking ends after 0.2 s at (0.02, 0); the remaining 0.3 s
allows at most 0.75 rad of steering. The controller returns a healthy Hold after
five reduced continuation validations. Independent execution remains clear of the
reviewed obstacle by approximately 1.44 mm. Stopping validation with retained
angles now covers all motion before subsequent stationary alignment.

Analytic regressions cover periods 0.1 and 0.5 s and rolling speeds 0.003, 0.1 and
0.2 m/s, including brakes that finish before, at and after the tick boundary.
The independent fixture integrates the braking portion separately, preserving
small residual displacement without quadrature across its zero-speed boundary.
It uses independent encoder formulas and does not advance via DriveModel,
TransitionModel or Kinematics::forward. The existing analytic complete-Brake and
moving-steering integration/sweep oracles remain intact.

## Confirmed Stable feedback

ModeExecutor now rejects Stable Drive if the latest measured mode is unconfirmed
or differs from its own stable actual mode. A prior healthy Hold cannot authorize
a later Drive against lost confirmation. The fault is latched and all rolling
targets are zero. Pending mode transitions continue their committed handshake and
mask incoming Drive as before.

Regressions begin with a healthy startup Hold and then introduce lost confirmation
or a mismatched actual mode for all three modes. TimedExecutor also rejects an
unconfirmed sample whose timestamps and command envelope are fresh, with no timing
error: fresh transport does not imply mode permission. The existing 720-case
transition matrix still checks deadlines, measured alignment and handover.

## Absolute speed throughout Drive

Checking only endpoint forward kinematics missed interior speed peaks as wheel
speeds and steering jointly interpolated. Independent dense encoder oracles expose
straightening DualAckermann, changing Ackermann curvature, opposing small Crab
steering errors and perturbed tangential Spin geometry. Endpoint-compliant raw
targets are rejected by the executor; model-generated bounded targets remain
accepted and preserve steering progress at a speed cap.

The certifier works over tick fraction f. For an affine signed wheel speed s and
angle a, its rolling vector has second-derivative norm bounded by
sqrt((2*ds*da)^2 + (max_abs_s*da^2)^2). Summing these bounds divided by four gives
body translation bound B; yaw's bound is B/module_radius. Triangle inequalities
can certify ordinary capped translation/rotation directly. Otherwise each
interval of width h uses a chord enclosure B*h^2/8; a derivative-sign bound proves
monotonic scalar components where possible. Midpoint checks reject observed peaks,
but sampling alone never grants certification. The budget is 4096 intervals and
maximum depth 14; exhaustion fails closed. The absolute-limit numerical allowance
remains 1e-9.

DriveModel checks the same full interval. If a cap cannot be certified, it reserves
rolling-speed margin in 1% target reductions while retaining desired steering
geometry and recalculating joint/body rate limits. Other rate/residual failures
halve the joint step. At most 60 refinements are allowed. This bounds work and
avoids freezing a turn by repeatedly reducing steering alone at an exact cap.
Controller stopping-continuation reductions remain a separate bounded mechanism.

## Duplicate-point curvature

Local curvature anticipation now walks nonzero geometric segments. Repeated
points do not discard either side of a bend. Navigation retains its existing
ordered-path and endpoint-heading behavior, including yaw-only endpoint intent.
The observing-critic regression repeats every radius-0.5 corner point one, two
and five times, checking every nominal, sampled and weighted Ackermann proposal
against the same yaw-rate speed budget; Crab retains its own translation budget.

A dedicated scurve_duplicates scenario repeats every S-curve point and runs all
five fixed seeds with the original behavior thresholds. Seed 1 peak tracking
error falls from the reviewed approximately 0.350822 m to 0.281948 m, below the
unchanged 0.3 m limit. No noise, lookahead, critic, completion or collision
threshold was relaxed.

## Verification

GNU 13.3.0, CMake 4.4.3, C++17, x86_64 Linux; -Werror:

| Check | Result |
| --- | --- |
| Release, benchmark enabled | 23/23 CTest entries passed |
| Debug, benchmark enabled | 23/23 CTest entries passed; final changed core suites rerun |
| AddressSanitizer + UndefinedBehaviorSanitizer | Seven core suites passed |
| Behavior matrix | Fourteen scenarios, five seeds each in Debug and Release |
| Installed consumer | Finds and links the 0.12 package |
| Allocation smoke | Existing straight/seed-42 peak-at-most-200 gate passed |
| Formatting/whitespace | clang-format and git diff --check passed |

Sanitizers use -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie and
-no-pie, with ASAN_OPTIONS=detect_leaks=0. Behavior, installation and benchmark
runs are excluded from sanitizers; no leak-detection claim is made. One local
navigation sanitizer executable lacked execute permission; restoring permission
within its test invocation and rerunning passed.

## Current Release measurement

The 80-run matrix contains fourteen behavior scenarios with five fixed seeds each,
plus five dense-arc and five distant-obstacle scaling runs. Seeds are 1, 7, 42,
73 and 101. Cases ran serially on a shared x86_64 host reporting AMD EPYC 9V74,
without CPU pinning. Defaults use a 0.1 s tick, 20-step horizon, 80 samples per
branch and two iterations. Timing includes the cold first compute call and
excludes constructor/setup, fixture actuation and CSV reporting.

The table reports completion ranges, the largest per-run planning P95 and peak
compute allocation count over each scenario's five seeds:

| Scenario | Completion (s) | Planning P95 (ms) | Allocation peak |
| --- | --- | --- | --- |
| straight | 3.5–3.7 | 32.420 | 113 |
| lateral | 7.4–14.7 | 24.317 | 117 |
| curve | 6.3–7.2 | 32.202 | 117 |
| spin | 4.3 | 0.000 | 16 |
| reverse | 3.5–3.8 | 29.898 | 113 |
| final_yaw | 6.6–6.8 | 29.042 | 113 |
| scurve | 12.4–13.0 | 30.281 | 118 |
| scurve_duplicates | 12.4–13.0 | 28.579 | 119 |
| cusp | 6.7–7.2 | 31.140 | 113 |
| loop | 20.1–38.2 | 30.395 | 113 |
| short_cusp | 2.5 | 0.000 | 13 |
| short_loop | 4.6 | 0.000 | 13 |
| short_corner | 2.9 | 0.000 | 13 |
| near_obstacles | 9.6–16.6 | 58.720 | 122 |
| dense_curve | 6.2–7.2 | 54.459 | 121 |
| obstacles | 6.3–7.2 | 58.828 | 118 |

All 80 cases completed. The largest observed compute time was 63.852 ms, with zero
100 ms period overruns; peak ordinary compute allocations were 122. Every row
has zero Waiting calls and zero safety-reduction validations. Dedicated safety
regressions establish reduction behavior; these default-workload measurements do
not bound worst-case retries or certification exhaustion. Shared-host timings do
not establish a hard real-time guarantee.

Plain and duplicate-point S-curves have identical completion time and peak error
for each seed, with maximum error 0.281948 m. Near-obstacle minimum measured
per-tick segment clearance is 0.100343 m after footprint/radius/margin inflation.
Measured chord clearance is a diagnostic, not a certificate for arbitrary
physical inter-tick motion; predictions separately validate swept enclosures.

Loop completion ranges from 20.1 to 38.2 s, compared with the historical 0.11
20.5–26.0 s range. The slowest new run still passes the unchanged 40 s completion
and three-second stall thresholds, but leaves only 1.8 s completion margin.
This is a remaining robustness margin to monitor across broader seeds and
calibrated dynamics; no acceptance limit was relaxed to obtain this result.

- [Core matrix, 70 runs](benchmarks/core_v0.12_release.csv)
- [Dense arc, five runs](benchmarks/dense_v0.12_release.csv)
- [Forty distant obstacles, five runs](benchmarks/distant_obstacles_v0.12_release.csv)

Reproduce with PERFORMANCE.md. The 0.11 and earlier records remain historical
evidence and have not been rewritten as measurements of the current source.

## Consumer migration

Use find_package(swerve_mppi 0.12 CONFIG REQUIRED). Public layouts and signatures
are unchanged from 0.11, but the phased Hold/RequestMode actuator contract changed.
Adapters must preserve full-period affine Drive targets and implement retained-angle
proportional braking before stationary alignment. Stopped thresholds must not
permit simultaneous steering with residual rolling speeds. See
[EXECUTION_CONTRACT.md](EXECUTION_CONTRACT.md) for feedback, actions and recovery.
Calibrated servo response, tire slip, ROS/simulator integration and hardware
validation remain subsequent work.
