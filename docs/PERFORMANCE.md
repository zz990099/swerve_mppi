# Core workspaces, diagnostics and measurement (0.6)

Stage 3 covers standalone core performance and observability. No simulator,
ROS adapter, transport protocol or hardware integration was added. Tracking
parameters, completion tolerances and execution interlocks retain their 0.5 defaults.

## Reproduce a measurement

Use an optimized build without concurrent builds or tests:

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS=-Werror -DSWERVE_MPPI_BUILD_BENCHMARKS=ON
cmake --build build-release --parallel 2
./build-release/swerve_mppi_benchmark all > core.csv
./build-release/swerve_mppi_benchmark dense_curve 42 > dense.csv
./build-release/swerve_mppi_benchmark obstacles 42 > obstacles.csv
```

`all` runs the nine behavior scenarios with seeds 1, 7, 42, 73 and 101. A named
scenario may use one seed or all five. The dense_curve case increases the same
2 m radius arc from 41 to 401 points; obstacles adds forty distant circular
obstacles to the regular arc. These stress computational scaling, not avoidance
quality. They are separate from `all` and the normal completion regression matrix.

For a controlled parameter experiment, the final positional arguments override
lookahead and path weight, for example:

```bash
./build-release/swerve_mppi_benchmark scurve 42 0.6 0.5
```

A task that faults or fails to complete within 400 ticks exits nonzero. The
benchmark shares the independent encoder fixture and scenario definitions with
behavior tests. It neither advances the plant with a controller prediction nor
changes configuration for speed. Constructor/setup and actuator/reporting work
are excluded from timing; the first compute call is included. Behavior tests
separately verify one second of post-completion Hold.

Each CSV row reports calls, completion time, maximum/RMS path error, mode changes,
nearest-rank P50/P95/P99 and maximum compute time, calls exceeding dt_s, ordinary
C++ allocation mean/peak, and planning-only P95/allocation mean. Planning-only
means compute calls that actually optimized at least one branch, including calls
that subsequently enter alignment or request a switch. Keeping that subset
separate avoids hiding MPPI cost behind cheap Hold/capture/transition calls.

The allocation instrument replaces ordinary new/new[] in the benchmark executable
only. It counts attempts that allocate successfully while compute executes; it
excludes constructor allocations, aligned allocation overloads and internal malloc
calls. The library does not contain allocator hooks or wall-clock instrumentation.

`--smoke` runs straight/seed 42 with defaults and rejects a peak above 200 ordinary
allocations per compute. CI enables the benchmark and runs this structural gate
through CTest. Wall-clock timing is reported, never used as a CI pass/fail threshold.

## Resource ownership

Optimizer owns reusable sample noise/masks, candidate/weighted controls and one
proposal trajectory. RolloutEngine supports both a value-returning API and an
output-parameter overload that retains vector capacity. Transition poses append
into that buffer. Results/fallbacks copy only data needed for returned solutions.
Controller retains capture/alignment safety models, critics, controls and trace.

Returned Solution/Trajectory values own their storage; subsequent solves cannot
mutate them. Copied controllers/optimizers own independent workspace vectors.
Serialize calls to each Controller/Optimizer. RolloutEngine's output buffer belongs
to the caller and must not alias its input controls. Invalid generation clears
previous validity/content, so stale successful traces cannot be scored again.
Reset restarts warm/RNG/task state without shrinking workspace capacity.

This reduces heap activity, but is not allocation-free: output/fallback ownership,
local references, branch/solution vectors and input validation still incur work.
Dense path cost remains proportional to local segment count per rollout pose;
obstacle cost remains proportional to obstacle count. No fixed solver deadline
or automatic wall-clock truncation is implemented. Host-specific real-time
budgets must be measured before using the core in a periodic adapter.

## Output diagnostics

ControlPolicy records the computation route taken in this call, independently
of Action and NavigationStatus:

| Policy | Route |
| --- | --- |
| Tracking | MPPI branch evaluation, possibly followed by alignment or a new mode request |
| Alignment | Continue an already committed same-mode alignment |
| Capture | Deterministic terminal/corner correction, possibly requesting a mode |
| ModeTransition | Continue a pending measured execution handshake |
| Stopped | A completed task is held/braked |
| Fault | SafeStop was emitted |

FailureReason is None for healthy outputs and distinguishes InvalidInput,
NonmonotonicTime, InvalidPath (path-processing failure), FeedbackFault,
NoFeasiblePlan, ModelFailure (immediate drive prediction), and TransitionFault
(deadline, handshake or request-ID failure). NoFeasiblePlan can arise from an
invalid rollout or a rejecting critic; it does not identify a particular obstacle
or individual critic. These fields explain the action and do not replace the
executor's own latched fault/recovery contract.

PlanningStats resets on every compute/optimize call:

- branches counts optimized branches, not only the winning branch.
- evaluated_rollouts includes the initial nominal, every sampled proposal and
  each evaluated weighted update; constant-intent safety checks are excluded.
- feasible_rollouts counts finite physical critic scores among those evaluations,
  before proposal-noise correction.
- fallback_updates counts iterations that return the existing/feasible fallback
  when weights or the weighted trajectory are invalid, including losing branches.

Legacy Output::feasible_rollouts remains the selected branch's finite sampled
proposal count across iterations. Large aggregate fallback counts can come from
infeasible losing switch branches; they do not alone imply an executor fault.
Capture, pending handshakes, completed tasks and invalid-input calls normally have
zero planning work. Counter identities and stopped-clock/infeasibility diagnostics
are covered by the optimizer regressions.

## Measurement evidence and tuning decisions

See VALIDATION.md and benchmarks/*.csv for the measured run, baseline and limits.
Allocation peaks are a repeatable structural comparison. Timing on a shared host
varies; these samples do not establish a hard real-time or worst-case guarantee.

Parameter probes used seed 42 with the original 0.5 core. Reducing lookahead from
1.0 to 0.6 m reduced S-curve peak error from 0.233 to 0.191 m, but increased arc
error from 0.054 to 0.114 m and S-curve completion time from 9.9 to 11.3 s. Raising
path weight from 0.5 to 2.0 at 1.0 m lookahead made the lateral task fail to complete
within forty seconds. The mixed 0.6 m / 2.0 probe reduced S-curve peak error to
0.187 m while increasing arc error to 0.132 m. These probes justify keeping the
defaults, not selecting a new universal parameter set.

Further standalone work should examine branch/entry-direction costs and tracking
objectives using these diagnostics, including blocked-path recovery and broader
curvature/direction cases. Retain all mode-confirmation/stop/collision gates when
changing proposals; a lower path error in one case is insufficient acceptance.
