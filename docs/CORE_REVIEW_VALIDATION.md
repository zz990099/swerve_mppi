# Ordered targets and controlled stopping validation (0.8.0)

Review baseline: 671083f52e8fa7a270a903f30a1ae214815436ef (0.7.0).
Measured core and benchmark source: a5c5602ba4a6cda9b276d17cfcb294807508dde9.
Recorded on 2026-10-01. No ROS, simulator or hardware integration was added.

## Corrections

PathManager supplies the effective translation target, target kind, arc gap to
that target and terminal eligibility. Uncaptured corners take precedence over
the global goal for speed limits and capture. Controller and GoalManager require
the same eligibility before terminal takeover or initial position acquisition.
The global goal remains the diagnostic/completion target; completion still needs
measured pose tolerances, stopped body/wheels, confirmation and continuous dwell.

Every healthy Brake/Hold/RequestMode passes one final stopping check against
fresh hard constraints. This covers terminal settling, yaw and mode-dwell waits,
completed-task external motion, local alignment and pending explicit handshakes.
A safe normal stop keeps its navigation state and immutable request. A rejected
or incomplete stop clears the request and emits UnsafeStoppingTrajectory/SafeStop.
NoFeasiblePlan remains a recoverable Waiting/Blocked stop when validation succeeds.

RolloutEngine::generate_stop checks braking with retained measured steering even
when mode confirmation is pending. It preserves actual mode, request ID and
confirmation; ordinary drive rollouts still require confirmed feedback. Models,
mode scheduling and the execution handshake retain their existing boundaries.

## Regression evidence

GNU 13.3.0, CMake 4.4.3, C++17, x86_64 Linux; -Werror:

| Check | Result |
| --- | --- |
| Release, benchmark enabled | 22/22 CTest entries passed |
| Debug, benchmark enabled | 22/22 CTest entries passed |
| AddressSanitizer + UndefinedBehaviorSanitizer | All seven core suites passed |
| Behavior matrix | Thirteen scenarios, five seeds each, both Debug and Release |
| Installed consumer | Finds 0.8 and exercises navigation eligibility and stop rollout |
| Allocation smoke | Existing straight/seed-42 peak-at-most-200 gate passed |
| Formatting/whitespace | clang-format and git diff --check passed |

Safety regressions additionally cover rejected terminal braking before settling,
yaw, translation, mode dwell and after completion; safe normal braking preserving
AligningGoal; injected constraints on stationary holds; and safe/rejected braking
during unconfirmed feedback with request identity preserved until cancellation.
Strengthened safety tests were rerun in all three build configurations.

Sanitizers use the existing non-PIE setup with detect_leaks=0 and cover core suites,
excluding behavior, installed-consumer and benchmark runs. One executable initially
could not start because its local execute permission was absent; restoring the
permission and rerunning passed. No leak-detection claim is made.

Three new default-configuration closed loops address the review reproductions:

| Scenario | Ordered path | Completion time, all five seeds |
| --- | --- | --- |
| short_cusp | (0,0), (0.2,0), (0,0) | 2.4 s |
| short_loop | Square with 0.1 m sides, returning to start | 4.4 s |
| short_corner | (0,0), (0.2,0), (0.2,0.08), (0.12,0.08); start at (0.12,0) | 2.7 s |

Tests require forward initial motion, ordered measured corner capture, measured
goal settling and a second of post-completion Hold. Capture uses the configured
0.06 m positional tolerance; these results do not imply exact waypoint passage.
The short cases use deterministic capture, so completion time is seed-invariant.

## Current Release measurement

Benchmarks ran serially after builds/tests completed, without CPU pinning on a
shared host. Defaults retain a 0.1 s tick, 20-step horizon, 80 proposals per branch
and two optimization iterations. Constructor/setup, actuator advance and reporting
are excluded from compute timing; the cold first compute call is included.

The core CSV contains 65 runs (thirteen scenarios by five seeds). Dense-arc and
distant-obstacle CSVs add five runs each, for 75 successful completions. Each row
contains all-call and planning-only timing, allocations, rollout work, Waiting
calls and measured inflated-circle clearance. A blank clearance means no obstacles.

Rows below show completion ranges and the maximum per-run planning-only P95 and
allocation peak over seeds 1, 7, 42, 73 and 101:

| Scenario | Completion (s) | Planning P95 (ms) | Allocation peak |
| --- | --- | --- | --- |
| straight | 3.8–3.9 | 18.093 | 84 |
| lateral | 7.1–8.9 | 20.342 | 86 |
| curve | 6.6–7.0 | 26.955 | 90 |
| reverse | 3.7–3.9 | 17.982 | 84 |
| final_yaw | 7.4–7.8 | 22.735 | 84 |
| scurve | 9.9–10.5 | 23.611 | 90 |
| cusp | 7.1–7.4 | 19.260 | 84 |
| loop | 20.7–25.7 | 19.828 | 84 |
| near_obstacles | 13.1–18.3 | 42.186 | 91 |
| dense_curve | 6.7–6.9 | 37.139 | 94 |
| obstacles (distant) | 6.6–7.0 | 46.084 | 91 |

Spin and the three short paths perform no MPPI optimization in these runs;
their planning-only P95 is zero and their full timing remains in the CSV.
Across all 75 runs, maximum observed compute time is 53.300 ms, ordinary allocation
peak is 94, and calls exceeding the 100 ms model period total zero. These shared-host
observations do not establish a worst-case deadline or target-machine performance.
No solve deadline or allocation-free operation was implemented.

near_obstacles puts forty radius-0.05 m circles 0.75 m either side of the regular
arc, giving nominal clearance 0.15 m after the default robot radius and margin.
All five tasks completed without faults or Waiting calls; minimum measured
per-tick-segment clearance across them is 0.108789 m after inflation. This checks
the independent encoder fixture, not Gazebo dynamics or tire slip. It validates
tracking through a feasible close corridor, not global detours around blocked paths.

Raw current evidence:

- [Core matrix, 65 runs](benchmarks/core_v0.8_release.csv)
- [Dense arc, five runs](benchmarks/dense_v0.8_release.csv)
- [Forty distant obstacles, five runs](benchmarks/distant_obstacles_v0.8_release.csv)

See PERFORMANCE.md for reproduction commands and metric definitions. Historical
0.6 CSVs and summaries in VALIDATION.md remain separate from these measurements.
README/ARCHITECTURE now identify standalone parameter assumptions instead of
claiming a bundled simulator or calibrated simulator agreement.

## Consumer migration and remaining boundaries

Rebuild consumers with find_package(swerve_mppi 0.8 CONFIG REQUIRED). Public
PathReference gains target, target_kind, target_remaining_m and goal_eligible;
standalone GoalManager callers must grant terminal eligibility explicitly.
RolloutEngine adds generate_stop. Serialized calls, clock/session recovery,
explicit measured mode feedback and independent actuator watchdogs still apply.

Static circular obstacles/footprint, per-tick swept segment approximation, one
switch per horizon, branch-seeded entry direction and uncalibrated actuator/slip
response remain model limits. This work does not add ROS, costmaps, transport,
simulation, a solver deadline or a new global navigation/recovery policy.
