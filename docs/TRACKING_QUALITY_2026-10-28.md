# Physical tracking follow-up, 2026-10-28

## Recovered checkpoint

The unfinished work is on `codex/tracking-quality`, originally at
`06d962afcc0d7bccfb54e5bfd4b0e8012eecc7c9`; `main` is still `821703b`.
The four tracking changes have **not** passed physical acceptance and must not
be described as a completed improvement or merged on the strength of portable tests.

Evidence:

- [Core/ROS run 40629970157](https://github.com/zz990099/swerve_mppi/actions/runs/40629970157):
  Debug, Release and ROS plugin contracts passed; physical following failed.
- [Recordings run 40629970200](https://github.com/zz990099/swerve_mppi/actions/runs/40629970200):
  all three rounds finished and uploaded CSV/JSON/PNG/logs. Seeds are 42/43/44.

The strict recorded result is **12/36 passed**, not 36 passing goals. A failed
quality gate counts as failure even if Nav2 returned success.

| Case | Seed 42 | Seed 43 | Seed 44 |
| --- | --- | --- | --- |
| straight | PASS | PASS | PASS |
| smooth_s | FAIL | FAIL | FAIL |
| smooth_curve | FAIL | FAIL | FAIL |
| crab | FAIL | FAIL | FAIL |
| crab_right | FAIL | FAIL | FAIL |
| reverse | PASS | PASS | PASS |
| reverse_curve | FAIL | FAIL | FAIL |
| combined | FAIL | PASS | PASS |
| spin | FAIL | FAIL | FAIL |
| final_yaw | FAIL | FAIL | FAIL |
| out_and_back | PASS | PASS | FAIL |
| closed_loop | PASS | FAIL | PASS |

## Findings and bounded fix

The plugin used `ChassisState.requested_velocity` as the slew anchor. A fresh
feedback message may carry a command older than the plugin's previous output.
Conditioning against that value does not constrain the sequence actually
published by Nav2. The follow-up retains the previous successful plugin output
for request conditioning, and keeps measured steering, wheel speeds and generator
state unchanged for prediction. Stop/reset/fault paths clear the output anchor.
The ROS regression deliberately supplies fresh feedback with an older, opposite
request between consecutive computes and checks the published request rates.

This addresses one source of normal-command discontinuity. It does not claim to
eliminate emergency zero commands after no-solution/faults, or solve the remaining
physical tracking failures.

The first round also shows:

- `smooth_curve` aborts with controller error 104 after repeated no-solution
  responses; `final_yaw` aborts with progress error 105 near its initial position.
- Lateral stopped fractions are 0.605/0.607 against the fixed 0.55 gate.
- Reverse-curve heading RMS is 0.0666 rad against 0.05 rad.
- Spin final truth yaw error is 0.0614 rad against 0.06 rad, despite goal success.
- S-curve and combined position RMS are 0.0302/0.0310 m against 0.03 m.

On the first-round lateral recording, local first-order response estimates from
the front-left measured/target steering and feedback timestamps have median
approximately 95 ms (153 intervals with nontrivial error and monotone response).
Targets can change within each interval, so this is diagnostic evidence, not a
calibrated actuator model. The nominal model at this checkpoint is 50 ms. A trial of a
100 ms nominal model with a 120 ms independent servo failed the unchanged stopped
fraction gate (0.685 for lateral motion). That trial was reverted; changing the
model constant alone is insufficient.

Reproduce the held-target, one-step audit without ROS:

```bash
python3 swerve_mppi_ros/tests/audit_steering_response.py \
  artifacts/round_01/crab/trajectory.csv \
  artifacts/round_01/smooth_curve/trajectory.csv
```

The tool uses feedback timestamps, removes duplicate source observations and
excludes faults, missing data and intervals outside 5--40 ms. It reports all four
steering joints together. For the two recordings above, 50/100 ms response models
yield RMS errors of 0.00759/0.00137 rad and 0.00616/0.00111 rad respectively.
This supports investigating servo lag; it does not validate complete MPPI rollouts.

## Remaining acceptance work

The output-anchor fix is `6730813`. Debug, Release, portable loops and the ROS
plugin regression passed in [run 40653719176](https://github.com/zz990099/swerve_mppi/actions/runs/40653719176).
The physical layer still failed. [Repeated run 40653719179](https://github.com/zz990099/swerve_mppi/actions/runs/40653719179)
finished with 4/12, 5/12 and 6/12 strict passes (15/36 total). This single repeat
does not establish a general performance improvement.

The new CSVs identify another command contract defect: the plugin sampled the
clock separately for `input.dt` and the output header. Under advancing simulated
time, a 0.045 m/s velocity increment was stamped only 25--30 ms after the previous
request (reported 1.8/1.5 m/s²). The follow-up uses one `control_stamp` for both
the control interval and the outgoing command header. This preserves timestamp
consistency; no quality gate is relaxed.

1. Verify the common-timestamp fix remotely and repeat the same 36 cases.
2. Replay timestamp-aligned actuator snapshots to identify servo response and
   prediction error; distinguish initial steering from repeated tracking stops.
3. Resolve no-solution recovery and the negative-heading translating case with
   the measured response, retaining collision checks and the existing gates.
4. Check terminal odometry/truth differences and request-rate failures after
   aborts. Keep the same scenarios, seed series and comparison provenance.
5. Merge to `main` only after the full physical acceptance passes. No PR is needed.

The subsequent A/B implementation is documented in [AB_IMPLEMENTATION.md](AB_IMPLEMENTATION.md).
