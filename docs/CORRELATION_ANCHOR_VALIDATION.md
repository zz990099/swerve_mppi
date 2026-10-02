# Correlated-noise and trajectory-anchor validation (0.13)

## Source and scope

Measured source: `7b9f315a490391c746f388b221760d73593d0616`.
This release fixes two P2 review findings in the standalone core. It changes no
tracking weights, noise parameters, completion limits, collision margins or
execution action contracts. ROS, simulation and hardware integration remain
outside this validation.

## Marginal temporal precision

Sampling evolves stationary AR(1) noise at every non-entry index, including
indices subsequently masked by same-mode alignment. The previous correction
incorrectly restarted its precision recurrence after every inactive interval.
For active indices i and j in the same sampling segment, marginal covariance is
sigma^2 * rho^abs(i-j). Correction now whitens successive active indices using
rho^(index gap), omitting inactive controls without discarding that covariance.
The frozen explicit mode-entry index is omitted and resets linkage, exactly as
in sampling. Optimizer passes its Branch explicitly to correction.

The reviewed rho=0.5 example with nominal vx [1,2,3], effective noise
[0.2,0.4,0.1] and mask [true,false,true] now scores 0.346666666667
with unit variance and weight, instead of 0.5. Independent dense covariance
elimination checks all 256 eight-index masks at rho 0, 0.5, 0.85 and 0.99,
with no switch, a middle switch and an initial switch. Further checks cover
separate translation/yaw variances, disabled dimensions, a generated alignment
mask and seeded reset reproducibility. Two 50,000-sample checks verify retained
lag-two correlation near 0.25 and independence across an explicit reset.

Projection and proposal-dependent masks still make this a practical weighting
heuristic. This change does not claim an exact importance sampler after clipping
or across discrete mode selection.

## Current-pose trajectory anchors

TrajectoryValidator now requires the first trace pose to match the measured
vehicle pose within 1e-9 m Euclidean distance and 1e-9 rad wrapped yaw. Invalid
anchors, including a nonfinite yaw difference, return Invalid before constraints
run. It also checks the exact current circular footprint independently of the
anchor tolerance, so a tiny tolerated displacement cannot hide current contact.

Regressions reject translated, rotated and stale caller-provided traces, accept
equivalent wrapped yaw and an anchored single-pose hold, and reject a current
collision even when a tolerance-close first pose is clear. The reviewed safe
remote trace [(2,0),(3,0)] cannot validate a vehicle colliding at (0,0).
Existing controller-generated rollouts, stopping traces and continuations remain
covered by the complete core and behavior suites. This is a pose/swept-circle
validator, not proof of dynamic reachability for arbitrary caller traces.

## Verification

GNU 13.3.0, CMake 4.4.3, C++17, x86_64 Linux, -Werror:

| Check | Result |
| --- | --- |
| Release CTest | 23/23 passed |
| Debug CTest | 23/23 passed |
| ASan/UBSan core suites | 7/7 passed, detect_leaks=0 |
| Final finite-anchor guard | All seven core suites rerun in all three builds |
| Installed consumer | Both builds passed; three- and four-argument correction linked |
| Extended Release behavior | 115/115 passed with unchanged limits |

Extended cases are loop seeds 2 through 100, plus near_obstacles and scurve
seeds 2,3,4,5,6,8,9,10. They include one second of settled Hold and checks for
mode faults, measured segment collision, tracking corridor and excessive stall.
Loop completion ranges from 19.1 to 39.4 s; seed 85 leaves only 0.6 s before the
40 s limit. All cases pass, but this narrow margin remains a robustness concern
for broader seeds and calibrated dynamics. No tolerance was relaxed.

- [Extended behavior, 115 runs](benchmarks/expanded_behavior_v0.13_release.csv)

## Serial Release measurements

All 80 runs completed: fourteen scenarios at seeds 1,7,42,73,101, plus five
dense_curve and five distant-obstacle cases. These ran serially without
concurrent builds/tests on an unpinned shared AMD EPYC 9V74 host. Defaults retain
a 0.1 s period, 20-step horizon, 80 samples per branch and two iterations.
Timing includes the cold first compute and excludes setup/fixture/reporting.

| Scenario | Completion (s) | Largest planning P95 (ms) | Allocation peak |
| --- | --- | --- | --- |
| straight | 3.5–3.8 | 32.186 | 113 |
| lateral | 7.8–9.1 | 23.497 | 117 |
| curve | 6.2–6.9 | 43.912 | 117 |
| spin | 4.3–4.3 | 0.000 | 16 |
| reverse | 3.5–3.8 | 29.568 | 113 |
| final_yaw | 6.6–7.6 | 29.594 | 113 |
| scurve | 12.4–12.7 | 27.843 | 118 |
| scurve_duplicates | 12.4–12.7 | 30.369 | 119 |
| cusp | 6.8–7.0 | 30.921 | 113 |
| loop | 20.7–27.5 | 34.928 | 113 |
| short_cusp | 2.5–2.5 | 0.000 | 13 |
| short_loop | 4.6–4.6 | 0.000 | 13 |
| short_corner | 2.9–2.9 | 0.000 | 13 |
| near_obstacles | 9.5–15.4 | 51.740 | 122 |
| dense_curve | 6.5–7.1 | 40.519 | 121 |
| obstacles | 6.2–6.9 | 45.168 | 118 |

The largest observed compute was 55.795 ms, with 0 period overruns and
peak ordinary compute allocations of 122. There were 0 Waiting calls and
0 safety reductions. Near-obstacle minimum measured chord clearance was
0.103460 m; largest plain/duplicate S-curve path error was 0.280933 m.
Measured chord clearance is diagnostic; predicted traces separately check swept
enclosures. Shared-host timings do not establish a hard real-time guarantee.

- [Core matrix, 70 runs](benchmarks/core_v0.13_release.csv)
- [Dense arc, five runs](benchmarks/dense_v0.13_release.csv)
- [Distant obstacles, five runs](benchmarks/distant_obstacles_v0.13_release.csv)

Reproduce with [PERFORMANCE.md](PERFORMANCE.md). Earlier CSVs and validation
records remain historical evidence.

## Consumer migration

Use `find_package(swerve_mppi 0.13 CONFIG REQUIRED)` and rebuild consumers.
`NoiseGenerator::correction(mean, noise, active, branch)` must receive the same
branch as sampling when it includes a mode entry. The default empty branch keeps
existing three-argument source calls valid for no-switch sampling; the exported
symbol signature changes, so old binaries must be relinked. Trajectory providers
must start at current feedback and regenerate cached traces when the pose changes.
The 0.12 phased residual-braking and whole-period affine Drive execution contract
remains in force; see [EXECUTION_CONTRACT.md](EXECUTION_CONTRACT.md).
