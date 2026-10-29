# A/B implementation, 2026-10-28

Status: implemented, published to validation branches, and physically recorded.
The full physical acceptance is not yet passing; no main merge has occurred.
The user explicitly authorized pushing the validation branches on 2026-10-29.
The chassis validation revision is published and pinned below; MPPI source
`f8e2d6a8b8fb71bb706c6753c1fb3a7054d373ad` is published to
`codex/tracking-quality`. Neither change is merged
to main. Do not interpret local tests as 36 passing physical cases. Stage C
(QP/via-state sampling) is not part of this change.

## A: recovery and rejection diagnostics

- Path/heading corridor violations incur finite normalized quadratic slack
  penalties. Collision, physical actuator limits, residual bounds and ordered
  checkpoint requirements remain hard checks.
- Every cycle includes a newly conditioned braking sequence from the current
  state. It goes through the same rollout and collision critics; an unsafe brake
  cannot be used to override a collision rejection.
- Diagnostics classify invalid input/state/control, actuator residual, generator
  failure, collision/custom critic rejection and critic/controller exceptions.
  Candidate counts use the first rejection reason. Failed computes are reported
  by the ROS plugin before Nav2 receives the failure.
- Soft tracking does not authorize skipping a leg. Predicted mandatory corners
  require 5 mm proximity before progressing; measured caller checkpoints retain
  their 15 mm gate. Corner approach is an explicit task objective to prevent
  horizon-end procrastination.

## B: executable reference and actuator response

`ReferenceProfile` retains independent path body yaw, including fixed-heading
lateral and reverse travel. Its scalar arc-speed envelope includes each wheel's
steering variation, wheel-speed variation, body yaw rate, servo tracking error
and forward/backward acceleration preview. Sharp corners are exact stop knots,
not smoothed steering changes spread into the incoming straight segment. Final
commands still pass through the shared generator and predicted residual checks;
the envelope alone is not a feasibility certificate.

Velocity tracking and a stationary steering-alignment objective prevent the
optimizer from avoiding an upcoming turn by parking indefinitely. Nominal
steering intent is held during stationary alignment. Reference motion pauses
only for a nonzero reorientation request capable of exceeding the stopped-wheel
threshold; a zero request cannot masquerade as presteering.

The shared chassis generator now advances stationary presteering targets at the
existing target-rate and joint limits. Both commanded and measured drive must
be stopped for this phase. Target lead is permitted while stationary; the
original measured steering-tracking gate still blocks driving until alignment.
The stalled-servo test continues to require zero drive. A separate 120 ms servo
test requires first drive within 1.5 simulated seconds without exceeding target
rates or starting drive before encoder alignment.

## Identification evidence

Source: `d3afb88`, physical recordings run `40654692871`, seeds 42/43/44.
Identification uses the first seed; the other seeds are held-out checks. Targets
are zero-order held at chassis source stamps, while encoder samples retain their
own source stamps; future targets are not substituted for earlier intervals.

| Seed | Case | Best nominal tau | Additional target delay | One-step steering RMS |
| --- | --- | --- | --- | --- |
| 42 | crab | 100 ms | 0 ms | 0.00138 rad |
| 42 | smooth_curve | 100 ms | 0 ms | 0.00124 rad |
| 43 | crab | 100 ms | 5 ms | 0.00123 rad |
| 43 | smooth_curve | 100 ms | 0 ms | 0.00106 rad |
| 44 | crab | 100 ms | 0 ms | 0.00148 rad |
| 44 | smooth_curve | 100 ms | 0 ms | 0.00135 rad |

The configured prediction time constant is therefore 100 ms. This identifies the
recorded target/encoder chain at its recording resolution, not tire dynamics or
unobserved target updates. Feedback age P99 in these cases is 5--15 ms; accepted
command age P99 is approximately 50--60 ms. Command age includes scheduling and
holding time and must not be interpreted as one-way transport latency.

New recordings retain independent `execution.csv` and observed `commands.csv`,
source stamps and command sequence IDs, as well as the existing truth/odometry
and prediction streams. Metadata separates initial wait to first rolling,
execution-phase durations and post-start stopped fraction; the original
full-route stopped-fraction gate remains unchanged.

Run the audit without ROS:

```bash
python3 swerve_mppi_ros/tests/audit_execution.py artifacts/round_01/crab/execution.csv
```

Use a case directory in the path when selecting one case. The audit also accepts
legacy `trajectory.csv`; it reports steering tau/delay candidates, source ages,
wheel-response error against the specified acceleration assumption, and
source-time-interpolated odometry/truth discrepancy when pose data is available.
Use `--wheel-acceleration` for a nondefault chassis drive limit.

## Local checks and remaining physical acceptance

- Shared chassis standalone randomized kinematics, target bounds, restore,
  transitions, stalled servo and finite-servo presteering tests passed.
- Release/Debug core and installed-consumer checks passed; portable caller and
  collision/braking contracts passed.
- 28 numerical loops (14 cases, 100/120 ms independent servo plants, steering
  rate at 80% of nominal) passed the existing position/heading, slew and stopped
  fraction gates. Maximum position RMS/peak: 0.00826/0.02750 m. Maximum stopped
  fraction: 0.3914. Optimized local compute P99: 38.01 ms. These are numerical
  tests on the current host, not independent physical or target-board evidence.
- Offline recording/calibration tests passed. Address/undefined-behavior
  sanitizer core checks passed with leak detection disabled: this traced host
  does not support LeakSanitizer, so leak checking is not claimed.

Published chassis validation branch: `codex/presteering-ab`, revision
`fd1c6b6cf8bfd5f818245e24cbbeb2f72985372b` (same source tree as local
`c1281b9218865fa08704fbd3557427881dbd017f`). Dependency and CI pins refer to
this published revision. MPPI algorithm source remains local `6b47462`; this
follow-up changes publication metadata and dependency identities only.

A separate 3 x 12 numerical visualization run used local `6b47462` and seeds
42/43/44: tracking gates passed 36/36, but host compute P99 <= 50 ms passed
only 11/36. These figures supersede neither physical acceptance nor the earlier
local test run; measured compute delays were not injected into the numerical
execution clock. The complete numerical results retain all timing failures.

Then run the unchanged 3 rounds x 12 Gazebo cases with seeds 42/43/44, inspect
full-route errors, recovery/braking counts, execution phases, duration and P99,
and compare against `d3afb88`. No main merge or physical improvement claim is
authorized by the local numerical results alone.

## Physical recording result — 2026-10-29

MPPI source: `f8e2d6a8b8fb71bb706c6753c1fb3a7054d373ad`.
Chassis source: `fd1c6b6cf8bfd5f818245e24cbbeb2f72985372b`.
The uploaded source trees match the prepared local source exactly.

[Repeated physical recordings run 40817964916](https://github.com/zz990099/swerve_mppi/actions/runs/40817964916)
completed all 36 cases and uploaded all three PNG/CSV/JSON/log artifacts.
All 36 Nav2 goals completed, but the unchanged strict gates pass **28/36**:

| Round | Seed | Strict pass | Goal success |
| --- | --- | --- | --- |
| 1 | 42 | 9/12 | 12/12 |
| 2 | 43 | 9/12 | 12/12 |
| 3 | 44 | 10/12 | 12/12 |

Maximum path RMS/peak across the 36 recordings: 0.020018/0.040766 m.
Maximum compute P99: 49.358188 ms. Maximum full-route stopped fraction: 0.448474.
The earlier `d3afb88` recording run passed 15/36 strict gates. This is one
matched scenario/seed comparison on different CI runners, not a statistical
performance guarantee or real-hardware validation.

Remaining strict failures:

| Round | Scenario | Failed metric | Value | Limit |
| --- | --- | --- | --- | --- |
| 1 | spin | final yaw error (rad) | 0.061149 | 0.06 |
| 1 | out_and_back | request linear rate (m/s2) | 1.1000 | 1.05 |
| 1 | closed_loop | request linear rate (m/s2) | 1.2375 | 1.05 |
| 2 | reverse | request linear rate (m/s2) | 1.1000 | 1.05 |
| 2 | spin | final yaw error (rad) | 0.061837 | 0.06 |
| 2 | closed_loop | request linear rate (m/s2) | 1.1000 | 1.05 |
| 3 | crab | request linear rate (m/s2) | 1.1000 | 1.05 |
| 3 | spin | final yaw error (rad) | 0.060763 | 0.06 |

[Core/ROS run 40817964971](https://github.com/zz990099/swerve_mppi/actions/runs/40817964971)
passed Debug/Release, formatting, offline audits, portable caller/closed-loop
contracts and ROS plugin lifecycle/state contracts. Its separate legacy physical
suite failed `spin`, `curve`, and `closed_loop`; those failures are not covered
up by the 36-case benchmark and need their own diagnosis.

[Chassis run 40817893811](https://github.com/zz990099/swerve_gazebo_sim/actions/runs/40817893811)
passed standalone/style/model tests plus both Humble/Fortress and Jazzy/Harmonic
independent physical acceptance.

The next implementation work is to audit source-stamped request rate violations,
terminal truth/odometry yaw mismatch and legacy curve/corner progress failures.
Preserve the current scenario set, seed series, thresholds and failed recordings.
Do not merge to main on the strength of the improved 28/36 result.
