# Stage 3 core engineering validation

Version: 0.6.0. Baseline: 129d8e63a5d8b09ab8c8ac09c4930ba9a99b40c7 (0.5.0).
[Stage 2](STAGE2_VALIDATION.md) and [Stage 1](STAGE1_VALIDATION.md) remain historical reports.
No ROS, Gazebo or hardware integration was performed.

GNU 13.3.0, CMake 4.4.3, C++17; warnings treated as errors:

| Configuration | Result |
| --- | --- |
| Release, benchmark enabled | 16/16 CTest entries passed |
| Debug, benchmark enabled | 16/16 CTest entries passed |
| AddressSanitizer + UndefinedBehaviorSanitizer | 5/5 core suites passed |
| Formatting/whitespace | clang-format and git diff --check passed |

The benchmark allocation regression joins the prior fifteen tests. CI enables it
in Debug/Release. Sanitizers use non-PIE executables, detect_leaks=0 and exclude
behavior/installed-consumer/benchmark runs; no leak-detection or new shared-library
result is claimed.

## Behavior preservation

All 45 default-noise runs retain the baseline completion tick and confirmed mode
changes. Reported maximum/RMS path errors agree within 1e-8 m. Both configurations
pass measured 0.06 m / 0.05 rad completion, body/wheel stopping and one second of
post-completion Hold. The existing 0.3 m corridor, three-second stationary bound
and four-switch bound remain unchanged. S-curve peak error is still roughly 0.25 m
and square completion still reaches 25.1 s; this stage does not claim better
tracking precision or corner traversal behavior.

## Allocation and timing measurements

The baseline executable links the 0.5 core and uses the same independent fixture,
scenario definitions, seeds and measurement loop. Current CSV adds planning-work
columns that were unavailable in 0.5. Setup/constructor and fixture/report work
are excluded; cold first compute calls are included. Full commands, metric
semantics and scope are in [PERFORMANCE.md](PERFORMANCE.md).

Rows below show the maximum per-run P95 and allocation peak over five seeds.
P95 includes cheap capture/alignment calls; current CSV also reports planning-only
P95. Timing comes from a shared Linux execution environment without CPU pinning;
no speedup ratio or worst-case timing guarantee is inferred from these runs.

| Scenario | Baseline allocation peak | Current allocation peak | Baseline P95 (ms) | Current P95 (ms) |
| --- | --- | --- | --- | --- |
| straight | 6648 | 84 | 22.392 | 19.042 |
| lateral | 6510 | 83 | 16.194 | 16.574 |
| curve | 6683 | 90 | 26.680 | 20.486 |
| spin | 22 | 7 | 0.018 | 0.015 |
| reverse | 6647 | 84 | 18.362 | 17.458 |
| final_yaw | 6645 | 84 | 19.897 | 19.397 |
| scurve | 6688 | 90 | 23.039 | 17.530 |
| cusp | 6648 | 84 | 17.540 | 17.711 |
| loop | 6648 | 84 | 17.537 | 16.858 |

Across the normal matrix, ordinary allocation peak falls from 6,688 to 90
(about 98.7%). The fixed straight/seed-42 regression permits at most 200; it has
84 in this run. No allocation-free or bounded-memory proof is claimed.

Normal-run maximum measured compute time: 28.392 ms. Measured calls beyond the 100 ms model period: 0.

| Stress sample (seed 42) | Calls | Completion (s) | P95 (ms) | Planning-only P95 (ms) | Allocation peak |
| --- | --- | --- | --- | --- | --- |
| dense_curve | 70 | 6.9 | 26.125 | 26.605 | 92 |
| obstacles | 70 | 6.9 | 23.377 | 24.078 | 89 |

The 401-point dense arc's P95 after workspace reuse but before nearest-distance
optimization was 64.303 ms; the final sample is 26.125 ms, with unchanged measured
behavior. That is an intermediate comparison, not the 0.5 baseline. The forty
obstacles are deliberately distant: this sample profiles query volume and does
not claim obstacle avoidance success. Both stress tasks complete in 6.9 s.

Raw evidence:

- [0.5 baseline, 45 runs](benchmarks/stage3-baseline.csv)
- [0.6 current, 45 runs](benchmarks/stage3-current.csv)
- [Dense arc before distance optimization](benchmarks/stage3-dense-before-distance-optimization.csv)
- [Dense arc, final](benchmarks/stage3-dense.csv)
- [Forty-obstacle sample](benchmarks/stage3-obstacles.csv)

## New boundary coverage

- Valid output-buffer reuse preserves storage/trajectory semantics; invalid reuse
  clears previous successful state. Returned solutions remain independent after
  another solve, and copied optimizer workspaces do not alias.
- Planning counters include initial/weighted evaluations and all executed branches;
  a rejecting collision retains zero feasible scores and exposes fallback work.
- Nonmonotonic time and infeasibility expose distinct failure reasons.
- The benchmark shares the behavior plant and definitions, while its allocator
  instrumentation remains outside the core library.
- Installed consumers request 0.6 and CI builds the optional benchmark gate.

Default tuning was retained after seed-42 probes showed conflicting S-curve/arc
results and a lateral failure under larger path weight. See PERFORMANCE.md for
the actual probes. Future standalone work should use the new counters to improve
branch/entry-direction costs and path fidelity. Simulator/adapter integration
remains deferred.
