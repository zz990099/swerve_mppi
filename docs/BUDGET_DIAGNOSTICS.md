# Offline pipeline diagnostics (0.21.2, CSV schema 2)

This stage instruments the offline measurement executable only. Public library
types, controller algorithms, mode protocol, execution guards and budget semantics
are unchanged. It adds no ROS/Gazebo wiring or physical acceptance.

## Reproduce and preserve evidence

Use a fresh optimized CMake build, then run without concurrent builds or tests:

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS=-Werror -DSWERVE_MPPI_BUILD_BENCHMARKS=ON
cmake --build build-release --parallel 2
python3 tools/run_budget_matrix.py \
  --executable build-release/swerve_mppi_integration_budget \
  --output-dir build-release/budget-recording --repetitions 50 --budget-ratio .6
python3 tools/run_budget_matrix.py \
  --executable build-release/swerve_mppi_integration_budget \
  --output-dir build-release/budget-acceptance --repetitions 200 --budget-ratio .6 --strict
```

The runner uses Python's standard library. It invokes the same CMake-built C++
executable for all nine cases: 41, 401 and 4096 path points, each with 0, 40 and
128 static circles. The path is the same radius-2 m arc, with a 0.7 rad span and
initial stationary snapshot. Circles use the same radius-2 m ring and 0.05 m
radii. Every repetition constructs fresh Controller, TimedExecutor and
ProfileRunner instances, with the resolved seed and full configured budget.
There is no warmup or warm-start reuse. This is a cold workload, not a moving
plant or a behavior completion test.

Version 0.21.2 resamples the original 0.7 rad curve for every density. Older
`--path-points` measurements used a shorter 0.5 rad curve while the default
fixture used 0.7 rad. Those sparse/dense timings confounded geometry and density;
do not reuse them as evidence for the normalized matrix. Both CSVs record the
actual input `path_span_rad`.

Add `--config profile.conf` to change samples, horizon, seed or other parameters.
The runner's explicit budget ratio wins over the profile. All selected workloads
must fit configured input limits; no points/circles are silently discarded.
An existing nonempty output directory is rejected so previous evidence survives.

Each directory contains:

| File | Contents |
| --- | --- |
| `summary.csv` | Nine distributions, work totals and acceptance counters |
| `calls.csv` | Every requested call, measured stages, actual work and error/status fields |
| `resolved.conf` | All 68 effective core values used by the pipeline |
| `manifest.json` | Tool version/compiler/build type; available CMake flags; source checkout SHA/dirty status; binary/input/artifact SHA256 hashes; OS/CPU-affinity metadata; exact argv and exit code |
| `stderr.txt` | Tool diagnostics, including configuration/output failures |
| `report.md` | Compact comparison table and explicit recording/acceptance status |

The runner verifies case coverage, repetition order/count and root outcome totals.
Strict and functional failures still preserve available evidence and return
nonzero. Invalid configuration may produce an incomplete report without traces;
it is never accepted as a completed matrix. CI uploads this directory even if
the measurement fails, while preserving the failed workflow result.
CI also renders the compact report in its job summary and logs.

The source entry describes the checkout at invocation; the executable hash
identifies the binary that ran. Build from that checkout immediately before the
measurement. A dirty checkout is not reproducible from its commit alone; preserve
its patch separately. CMake cache details are included when found beside the
binary. CPU load, governor, affinity changes and deployment middleware workload
require operator control beyond these automatically recorded details.

The original CLI remains available:

```sh
./build-release/swerve_mppi_integration_budget 50 --budget-ratio .6 \
  --trace calls.csv --write-config resolved.conf > summary.csv
./build-release/swerve_mppi_integration_budget 50 --config resolved.conf \
  --path-points 401 --obstacles 40 --trace selected.csv > selected-summary.csv
./build-release/swerve_mppi_integration_budget --build-info
```

Without selection flags it still runs the original 41-point, 0/40/128-circle
workload. `--path-points` accepts 2..4096; `--obstacles` accepts 0..128. `--matrix`
cannot be combined with either selection flag. Trace output must differ from
input/resolved configuration paths, including existing symlink/hardlink aliases.
CSV values use the classic locale and round-trip double precision.

## What is measured

All wall times use `steady_clock`. Consecutive boundaries partition each call:

| Stage | Work | In live pipeline budget? |
| --- | --- | --- |
| `setup` | Fresh consumers and pre-call measurement bookkeeping | No |
| `controller` | Entire Controller::compute: input/path preparation, indexed geometry, all optimized branches, selection, safety and output conversion | Yes |
| `envelope` | Output copy and originating-task snapshot capture | Yes |
| `admission` | Chassis compilation, timing/task/protocol checks and execution interval/stop validation | Yes |
| `install` | Checked profile installation | Yes |
| `sample` | Initial sample if installation succeeded; otherwise the conditional check | Yes |
| `endpoint_sample` | Additional nominal end-of-interval sample, if installed | No |

The pipeline is the measured interval from before Controller::compute through
the initial sample. It equals the sum of the five included stage times for each
call. Setup, destruction, endpoint verification, aggregation and file I/O are
excluded. Raw records are reserved before the repetitions and written after each
workload group. Clock probes add a small measurement cost; none is added inside
optimizer evaluations or production library callbacks.

Summary P50/P95/P99/max are nearest-rank percentiles. Each stage has its own P95;
controller timing also has P50/P99/max. Percentiles from different stages cannot
be added to recover pipeline P95: slow stages may occur in different calls.
Use `calls.csv` for correlated investigation. Legacy leading summary columns are
preserved; new diagnostics are appended with unique names.

`pipeline_cpu_ms` uses process CPU ticks from `std::clock`. Unsupported, negative
or wrapped samples remain blank, and `cpu_samples` reports coverage. The signed
`wall_minus_cpu_ms` gap can be slightly negative from resolution/probe placement.
A large positive gap suggests time not accounted for by process CPU work; it does
not prove an OS scheduling stall or explain its cause. Process CPU may include
other threads. This clock does not drive any safety or acceptance decision.

## Work and failure accounting

Raw records export the existing `PlanningStats` values: branches actually
optimized, evaluated rollouts (nominal, sampled and weighted), finite feasible
rollouts, fallback updates and budget exhaustion. Safety reductions are recorded
separately. Summary totals include partial work from timed-out calls. No counter
is inferred by multiplying configured samples, iterations or horizon. Capture,
transition and stop work remain outside these optimization counters; inspect
`control_policy` to distinguish routes.

Every call has one root outcome, evaluated in pipeline order:

| Outcome | Meaning |
| --- | --- |
| `ok` | Authorized controller result, healthy admission, installed profile and both samples |
| `compute_timeout` | Explicit controller timeout with all command/profile/sample authorization withheld |
| `controller_failure` | Other failure reason or absent command; also any authorization after a timeout |
| `admission_failure` | Timing/safety rejection, nonvalid rejected trajectory, missing profile or executor fault, even when a safe fallback installs |
| `installation_failure` | Healthy admitted profile did not install |
| `sampling_failure` | Initial or nominal endpoint sample missing after installation |

Raw `failure_reason`, `timing_error`, `safety_error`, `rejected_status`, authorization
flags and policy remain available even after a root outcome is chosen. A safe
fallback protects the plant but does not establish successful execution of this
benchmark workload. Functional failure counting now counts failed calls once,
rather than counting multiple events in one call.

`compute_timeouts` counts explicit timeout reasons. `total_overruns` counts full
pipeline time strictly greater than `dt_s * compute_budget_ratio`. These counts
can overlap, and a rejected timeout can still consume less than the total budget.
Overruns are also partitioned into controller time already over budget and
post-controller overruns whose controller time fit. These identify a measured
location, not a causal explanation. Root outcomes partition calls; an unsafe
timeout can count as both an explicit timeout and a functional controller failure.

Recording mode exits nonzero for any functional failure and records timeouts or
overruns without claiming budget acceptance. `--strict` additionally rejects any
timeout or pipeline overrun. `recorded_no_failures` reports this observed sample
only, not a worst-case guarantee. Cooperative timeout timings are truncated solves,
not unconstrained optimizer performance; inspect timeout counts before comparing
percentiles or sample/horizon profiles.

## Acceptance boundary

Deterministic report tests cover time partitioning, percentile ranks, unavailable
CPU clocks, root failure precedence, timeout authorization, exact budget boundaries
and strict/recording decisions. CLI tests compare summary values to an independent
per-call CSV oracle, check workload/configuration selection, validate manifests and
hashes, preserve failed strict reports and reject output aliases/overwrites.
Existing behavior and installed-consumer regressions remain enabled.

The fixed fixture keeps source/application timestamps coherent and unchanged while
CPU work executes; it does not age queued commands by measured wall time. It cannot
validate transport deadlines, actual executor scheduling, slip/localization or
physical stopping. Controller time does not isolate internal rollout/critic cost.
Use density and configuration scaling to select the next profiling experiment.
Run strict acceptance on the intended CPU under the intended competing workload,
then perform separate transport and plant acceptance after the preparation stages.
