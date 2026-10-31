# Local numerical tests without ROS

The local superbuild compiles the actual MPPI core and shared chassis executor.
An independent first-order steering plant closes the loop at 100 Hz; MPPI runs
at 20 Hz. This is a numerical test, not a replacement for ROS lifecycle, TF,
odometry, costmap integration, Gazebo contact dynamics or physical real-time gates.

Requirements: a C++17 compiler, CMake >=3.20, Python 3 and
`python3 -m pip install -r tools/requirements-local.txt`.
Use `swerve_chassis_core` from `zz990099/swerve_gazebo_sim` at
`8f986bda64d5b48af012d37cf924328e0b127468`, matching CI.
The source directory can be passed directly or via its repository root.

From the MPPI repository root, build incrementally and run one scenario:

```bash
python3 tools/local_test.py --chassis-root ../swerve_gazebo_sim \
  --case crab_right --output artifacts-local/crab-right
```

Run a selected subset instead of all 32:

```bash
python3 tools/local_test.py --chassis-root ../swerve_gazebo_sim \
  --case crab crab_right reverse_diagonal lateral_zigzag figure_eight \
  --output artifacts-local/regressions
```

Defaults match the long-path tests: minimum translational route length 12 m,
1 m/s planning cap, seed 42, 0.12 s steering servo, 130 s simulated budget.
Use `--minimum-length 0 --max-speed 0.8` for original route sizes,
`--servo-time 0.10` for the nominal plant, and `--max-time` for a shorter budget.
`--cmake /absolute/path/cmake` and `--build-dir` support isolated environments.
Each output directory must be new, to avoid mixing recordings from different runs.

After the first build, run the binary directly through the Python recorder:

```bash
python3 swerve_mppi_ros/tests/run_portable_recordings.py \
  --executable build-local/record_portable --output artifacts-local/final-yaw \
  --cases final_yaw --no-plot
```

Explicit `--all-cases` selects all 32. The default is only `straight`.
`--keep-path-yaw` is for historical ablation binaries; it bypasses preparation
in the harness and does not change the ROS product's external-path contract.

Outputs: per-case CSV trajectories, original references, `metrics.csv`,
`experiment.json`, `run.log`, and overview/detail PNGs. Functional `passed`
requires goal pose/stopped/settling/zero-request conditions plus <=10 cm
whole-trajectory XY deviation. It does not imply strict heading, stopped-fraction,
physical timing or complete CI acceptance. Read `metrics.csv`; the recorder's
process exit status only detects infrastructure errors, not failed scenarios.
P99 measurements under concurrent local jobs are diagnostic only.

Build and run the existing contract and quality checks separately:

```bash
cmake --build build-local --parallel 2
ctest --test-dir build-local --output-on-failure
ctest --test-dir build-local -R '^(core|heading_reference|liveness|support_tests|installed_consumer)$' \
  --output-on-failure
```

`closed_loop` and `stagnation` retain their original acceptance thresholds.
They are currently known failures: see `GEOMETRIC_HEADING_LOCAL_OPTIMIZATION.md`.
The installed-consumer test installs both packages from the superbuild, relocates
the installation, and builds a separate consuming project.

Compare three compatible recording sets:

```bash
python3 swerve_mppi_ros/tests/plot_local_comparison.py \
  --main artifacts-local/main --before artifacts-local/before \
  --after artifacts-local/after --output artifacts-local/comparison.png
```

Historical four-group ablation (separate worktrees, no algorithm edits there):

```bash
git worktree add ../mppi-main35 35b082b9fc087b0598f9d6e3a845aa9ea23c1246
git worktree add ../mppi-before cf7a63b24ee8fe12e1b98180197098edff6e7ef0
python3 tools/local_ablation.py --main-source ../mppi-main35 \
  --previous-source ../mppi-before --chassis-root ../swerve_gazebo_sim \
  --output artifacts-local/ablation
```

This builds main, a geometry-only caller adapter against main, and the previous
PR against original/geometric intermediate yaw. It runs the same seven selected
cases, sequentially. `--cases straight` is a fast build/adapter smoke check.
The geometry-only adapter preserves the original final yaw as a separate goal;
its incompatibility with the baseline terminal logic is part of the experiment.
Do not interpret this adapter as an exact source-level feature-toggle factorial.

## Decision audit

Enable only for diagnosis; CSV I/O and per-candidate matching affect timing:

```bash
python3 swerve_mppi_ros/tests/run_portable_recordings.py \
  --executable build-local/record_portable --output artifacts-local/figure-eight-audit \
  --cases figure_eight --decision-log --no-plot
python3 tools/analyze_decisions.py \
  artifacts-local/figure-eight-audit/figure_eight_decisions.csv --time 30 \
  --output artifacts-local/figure-eight-decision.json
```

The optional 56-column decision CSV records every iteration/candidate and the
weighted update: raw/conditioned controls, rejection reason, predicted ordered
progress, cost breakdown, measured velocity/pose/steering, requested motion and
steering targets. Candidate indices: 0 continuation, 1 fresh, 2 brake, 3 slow,
4 short recovery, 5 guided, 6 reconsidered intent, 7 terminal stop/rotation,
8 terminal position, >=9 exploration; index `samples` is the weighted update.
Unavailable specialized proposals fall back to fresh/mean/guided controls.
The original 18-column trajectory format stays unchanged. Production diagnostics
are off by default; `Input::record_decisions` must be explicitly enabled.
The analyzer inspects captured frames without rerunning the scenario and maps
non-finite cost values to JSON null. It ignores incomplete trailing rows.

`liveness` freezes two measured stopped states from the earlier failed recordings
and checks that feasible useful motion wins against parking, optional auditing
preserves output, terminal rotation predicts braking first, cached preferences
stay valid and independent of crop length, retained retiming context is used,
and hard critic rejection still wins.
