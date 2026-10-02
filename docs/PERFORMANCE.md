# Core workspaces, diagnostics and measurement (0.12)

Stage 3 covers standalone core performance and observability. No simulator,
ROS adapter, transport protocol or hardware integration was added. Tracking
parameters, completion tolerances and execution interlocks retain their 0.5 defaults.
Version 0.12 adds whole-period absolute speed certification, phased residual
braking/alignment and duplicate-point curvature handling. Current measurement
evidence is in ACTUATION_BOUNDARY_VALIDATION.md and benchmarks/*v0.12_release.csv.
The 0.11 DRIVE_EXECUTION_VALIDATION.md and CSVs remain historical evidence. The 0.8 CSVs
and stage 3 tuning conclusions remain historical evidence; their timings and
completion ticks do not describe the current source.

## Reproduce a measurement

Use an optimized build without concurrent builds or tests:

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS=-Werror -DSWERVE_MPPI_BUILD_BENCHMARKS=ON
cmake --build build-release --parallel 2
./build-release/swerve_mppi_benchmark all > core.csv
./build-release/swerve_mppi_benchmark dense_curve 42 > dense.csv
./build-release/swerve_mppi_benchmark obstacles 42 > obstacles.csv
./build-release/swerve_mppi_benchmark near_obstacles 42 > near.csv
```

`all` runs fourteen behavior scenarios with seeds 1, 7, 42, 73 and 101: the original
nine, three short ordered paths, near_obstacles and scurve_duplicates. The duplicate
S-curve repeats every geometric point and uses the same acceptance thresholds. A named
scenario may use one seed or all five. The dense_curve case increases the same
2 m radius arc from 41 to 401 points; the legacy obstacles name adds forty distant
circles to the regular arc. These stress computational scaling and remain separate
from `all`. near_obstacles places forty circles in two rows 0.75 m either side of
the regular arc, leaving 0.15 m nominal clearance after radius and margin inflation.
It exercises collision rejection and clearance costs close to measured motion.
Its five-seed completion and positive measured segment clearance run in CTest.
This corridor does not establish global detour planning around a blocked path.

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
Two additional columns report Waiting calls and minimum inflated-circle clearance
along measured per-tick motion segments. Clearance is blank when there are no
obstacles. It checks the independent fixture observations, not a predicted rollout;
continuous-time dynamics and noncircular footprints require independent validation.

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
Controller retains its safety rollout engine and a reusable continuation trace. In 0.7
hard validation is shared with Optimizer instead of a separate safety CriticManager.

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
| Blocked | A checked planning stop awaits a fresh feasible plan (0.7) |

FailureReason is None for healthy outputs and distinguishes InvalidInput,
NonmonotonicTime, InvalidPath (path-processing failure), FeedbackFault,
NoFeasiblePlan, ModelFailure (immediate drive prediction), and TransitionFault
(deadline, handshake or request-ID failure). Version 0.7 adds UnsafeStoppingTrajectory
for rejected/incomplete stopping predictions. NoFeasiblePlan can arise from an
invalid rollout or a rejecting critic; it does not identify a particular obstacle
or individual critic. These fields explain the action and do not replace the
executor's own latched fault/recovery contract.

PlanningStats resets on every compute/optimize call:

- branches counts optimized branches, not only the winning branch.
- evaluated_rollouts includes the initial nominal, every sampled proposal and
  each evaluated weighted update; capture/alignment/stopping safety checks are excluded.
- feasible_rollouts counts finite physical critic scores among those evaluations,
  before proposal-noise correction.
- fallback_updates counts iterations that return the existing/feasible fallback
  when weights or the weighted trajectory are invalid, including losing branches.

Legacy Output::feasible_rollouts remains the selected branch's finite sampled
proposal count across iterations. Large aggregate fallback counts can come from
infeasible losing switch branches; they do not alone imply an executor fault.
Capture, pending handshakes, completed tasks and invalid-input calls normally have
zero planning work. Output::safety_reductions and the CSV safety_reductions column count reduced-intent
continuation validations; these are excluded from optimization rollout counts.
Counter identities and stopped-clock/infeasibility diagnostics
are covered by the optimizer regressions.

## Measurement evidence and tuning decisions

See ACTUATION_BOUNDARY_VALIDATION.md and benchmarks/*v0.12_release.csv for the current
measurement, source commit, environment and acceptance results. CORE_REVIEW_VALIDATION.md
records the historical 0.8 matrix; VALIDATION.md records stage 3 tuning probes.
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
objectives using these diagnostics, including broader blocked-path and
curvature/direction cases. Retain all mode-confirmation/stop/collision gates when
changing proposals; a lower path error in one case is insufficient acceptance.

The tuning measurements above are historical 0.6 evidence. Version 0.7 changes path matching
and capture safety semantics; it does not claim preservation of every 0.6 completion
tick or reuse its timing values as new measurements. Its allocation smoke gate and
behavior matrix are rerun; see PRE_SIMULATION_VALIDATION.md.
Version 0.8 has a fresh committed measurement matrix; see CORE_REVIEW_VALIDATION.md.
