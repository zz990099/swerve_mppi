# A/B implementation, 2026-10-28

Status: implemented and locally tested. ROS/Gazebo physical acceptance is pending.
The user explicitly authorized pushing the validation branches on 2026-10-29.
The chassis validation revision is published and pinned below; this MPPI revision
is prepared for publication to `codex/tracking-quality`. Neither change is merged
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
