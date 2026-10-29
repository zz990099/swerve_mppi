# Continuous MPPI validation

Old baseline: MPPI `7a1f912a557410056bcfea9d34d2a704e2b563ae` with chassis
`9358c96884d7fa4866cea7c053daa7a7cec82d8f`. New chassis pin is in dependencies.repos.
No old/new physical performance improvement is claimed without matched recordings.

## Automated layers

1. Standalone core: shared next-step target equivalence, continuous translation
   plus rotation, lateral presteering, invalid snapshots, invalid input, injected
   no-solution critics, zero planner limit and installed CMake consumer.
2. Portable caller contracts: ordered progress, closed/out-and-back path handling,
   stopped goal checks, grid bounds, footprint edge and between-sample collisions.
3. Portable closed loops: straight, lateral, reverse, spin, combined movement,
   S curves, square and return paths, with execution steering rate reduced to 80%
   of nominal, with independent 100 ms and 120 ms servo plants against the
   100 ms prediction. Fourteen scenarios per plant include negative final heading.
   These are numerical model tests, not independent physical evidence.
4. ROS plugin tests: plugin loading/cleanup, lifecycle, speed caps, continuous
   output, fault zero/recovery, physical configuration mismatch, TF/map errors,
   stale input and ordered goal completion.
5. Gazebo/Nav2 FollowPath: eleven independent physical scenarios including combined
   vx/vy/wz, fixed-heading S curve and live action cancellation, measured by independent Gazebo poses.
   Endpoint gates are 0.06 m and 0.06 rad; ordered checkpoints are required for
   return/closed paths. Each case logs path RMS/peak, stopped sample fraction,
   wall duration and compute percentile diagnostics.

Local host has no ROS/Gazebo. ROS and physical acceptance must be established by
remote CI, not inferred from portable prediction tests. CI uses Jazzy/Harmonic;
this is not Humble Nav2 certification. Target CPU real-time performance, moving
obstacles, real hardware, calibrated tire slip and long-term drift remain outside
this acceptance. The conservative footprint critic has numerical collision tests;
obstacle navigation is not included in the physical tracking scenes.

Run `python tools/format.py --check` with the pinned Rolling ament tools. See CI
for exact standalone dependency installation and colcon commands. Version 1.0
removes the Python executor comparison because that executor no longer exists.

## Physical trajectory recordings (32 cases, one round)

With the workspace built and sourced, run:

```bash
python3 swerve_mppi_ros/tests/run_trajectory_rounds.py \
  --chassis-root /path/to/swerve_gazebo_sim --output artifacts --rounds 1 \
  --seed-base 42 --min-path-length 12 --max-speed 1.0 --path-frame map
```

Install Python matplotlib, PyYAML and psutil. The default is one round with
fixed seed 42. It independently launches 32 fresh Gazebo/Nav2 cases, including
20 added diagonal, corner, zigzag, U-turn, circular, figure-eight, slalom,
spiral, cloverleaf and body-rotation trajectories. See
[TRAJECTORY_CASES.md](TRAJECTORY_CASES.md) for the full ordered list.
Use `--cases` to select a subset and `--seed-base` to change the fixed seed.
An existing round directory is rejected rather than overwritten.

Every case streams independent Gazebo observations and wheel odometry to
`trajectory.csv`, with original source timestamps and requested velocity. A fixed
startup transform expresses both observed tracks and the reference in the initial
body frame; this makes perpendicular/reverse starts explicit. Body-yaw arrows
show orientation independently of travel direction. `result.json` retains the
reference, seed, status, errors and compute percentiles. Interrupted/failed cases
retain partial data and remain marked FAIL in the montage. A round produces one
`round_NN.png`, four eight-case detail pages and a `summary.json`; no simulated/reconstructed track is substituted
for missing observations. Re-render without ROS using:

```bash
python3 swerve_mppi_ros/tests/plot_trajectories.py artifacts/round_01
```

The recordings workflow runs one serial round of all 32 cases with seed 42;
there is no round/seed matrix. It uploads the PNGs, CSV, JSON and process logs.
Core and ROS interface checks remain in the other workflow, which no longer
repeats the old eleven-case physical run. These are simulated physics tests,
not hardware trials or a statistical reliability estimate.

All translational routes have 12 m reference length in CI. Interior checkpoints
at 0.75 m arc-length spacing must be visited in order using independent truth
(within 0.20 m), so a closed or self-crossing path cannot pass simply by ending
near its start. Action success without ordered route completion is a failure.

## Tracking quality gates (fixed before the new physical comparison)

All physical cases now record actuator feedback, source/command times, selected
predictions and compute/cost/effective-sample diagnostics. A successful action
alone is not sufficient. `tracking_quality.py` enforces:

| Metric | Limit |
| --- | --- |
| Full-path position RMS / peak | 0.03 m / 0.075 m |
| Body-heading RMS / peak along translating references | 0.05 rad / 0.12 rad |
| Endpoint position / heading | 0.06 m / 0.06 rad |
| Fraction of observations with all wheels below 0.05 rad/s | 0.55 |
| Accepted-request linear / angular rate | 1.05 m/s² / 1.5 rad/s² |
| Core compute P99 on the CI CPU | 50 ms |
| Duration | max(8, 5 + 4 max(path_length/0.8, yaw_travel/0.8)) seconds |

In-place rotation has no unique heading as a function of geometric path position,
so its along-path heading RMS/peak is omitted; endpoint heading remains gated.
Cancellation omits endpoint gates and still requires a canceled action and stopped
measured wheels. Failures retain raw data and remain FAIL in each montage. The
28 numerical loops additionally gate position/heading errors, request slew and
stopped fraction for rate-limited and slower first-order servo plants. The prior
36-case recordings at 543613e are the comparison baseline; seeds 42/43/44 and all
reference paths remain unchanged. Target-board timing still requires measurement.

See [A/B implementation and local evidence](AB_IMPLEMENTATION.md) for the
unpublished follow-up. Portable loop success does not establish ROS/Gazebo acceptance.
