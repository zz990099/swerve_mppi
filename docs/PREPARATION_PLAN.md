# Staged implementation plan

Target boundary: a ROS-independent planner returns body velocity and explicit mode
requests; the independent Gazebo chassis executes those requests. No old execution
or transport interface will receive compatibility wrappers. Complete one stage per
iteration, test it, and push directly to main before starting the next stage.

| Stage | Status | Deliverable and acceptance |
| --- | --- | --- |
| 1. Planning/execution ownership | Complete in 0.22 | Remove production executors, profile sampling, session/sequence and peer adapter contracts; privatize prediction commands; remove obsolete companion preparation tools; preserve planning regressions, nominal closed-loop behavior and installed-consumer checks |
| 2. Current chassis prediction | Complete in 0.23 | Match Python speed/steering limits, normal zero, brake-align-confirm transitions and same-mode realignment; preserve mode dwell/hysteresis and measured acknowledgement; add offline parity tests against current chassis semantics |
| 3. Observation, timing and faults | Complete in 0.24 | Use integer-nanosecond source time; separate observation/decision/publication/application/expiry; bound pairing, age, motion evidence and history; latch runtime faults; recover deliberately; separate planning/model/chassis periods and elapsed warm-start shifts |
| 4. Current interface offline adapter | Next | Map rad/s to m/s, modes, request IDs, frozen receipts and mode age; retain accepted ID/entry on ordinary drive/hold; reject stale outputs; compare only shared chassis parameters; test message sequences without running ROS/Gazebo |
| 5. ROS and physical integration | Pending | Separate algorithm-side ROS package; validate straight/crab/spin and transitions, then paths, obstacles, stopping and delay; add Nav2 wrapper after basic closed-loop acceptance |

## Completed: stage 2

Replace the old interpolated joint/proportional-brake laws with current Python body
saturation, independent wheel ramps, zero hold and brake/align/dwell prediction. Carry
command-side memory through rollouts, preserve raw entry curvature, nearest-angle tie
order and immutable accepted receipts. Keep mode age, switch cost and hysteresis.
Handle observed automatic same-mode realignment using the issued entry intent.

Acceptance includes 7,960 direct Python command-cycle comparisons, six directed
mode changes, two wheel cap/ramp sets, reversals, zero and same-mode realignment;
core held-measurement and stopping/sweep tests; all nominal closed-loop scenarios;
independent lag/slip/noise probes and the installed consumer. No physical ROS/Gazebo
wiring is included. Cold command-history initialization remains an explicit assumption.

## Completed: stage 3

Version 0.24 removes floating source stamps and the dual feedback ingress. Joint,
pose, mode and independent-motion observations retain integer source time and pair
within configured skew/age/future bounds. Controller outputs distinguish observation,
decision, publication and expiry; the following input reports publication and a
bounded application interval without relabelling either observation.

Encoder/body numerical consistency remains a strict nominal model gate. Independent
motion evidence has a separate source, error envelope and admission policy; admitted
residual bounds expand trajectory clearance over time. Exact known command application
reconciles hidden chassis prediction memory against measured joints. Missing or
uncertain history is diagnosed and cold-seeded; invalid identity/time or measured
divergence latches a fault.

Planning, model and chassis periods are independent configuration fields. Warm starts
shift by elapsed model intervals. Timestamp replay/rollback, stale observations,
invalid application history, feedback/motion violations, compute timeout and mode
transition failure withhold commands and latch until `Controller::recover` accepts a
newer coherent sample. Dedicated timing/lifecycle regressions cover these contracts.

## Next: stage 4

1. Define algorithm-side offline wire DTOs matching only the current Python chassis
   command/state schema; do not add ROS dependencies or restore deleted protocols.
2. Convert wheel joint rad/s to core rolling m/s and map current mode, confirmation,
   fault, request ID, accepted entry/geometry and actual-mode age into `VehicleState`.
3. Map `Output` to current velocity/mode commands. Preserve the accepted request
   receipt on ordinary drive/hold, stamp publication once, and reject expired output.
4. Report the exact/bounded chassis application interval back as `CommandApplication`;
   reject mismatched IDs, time domains, stale state and invalid transition sequences.
5. Compare only shared chassis parameters and fail startup on incompatible wheel
   geometry/limits/rates/tolerances. Planner-only parameters remain algorithm-owned.
6. Add transport-free sequence tests for startup, drive, zero, all mode changes,
   retries, same-mode realignment, expiry, delay, duplicate/out-of-order state and
   deliberate recovery. Do not launch ROS or Gazebo in this stage.

## Remaining sequence

Stage 4 adds only the current command/state mapping and offline sequence tests. It
must preserve request identity, accepted geometry and source-result lifetime while
supporting normal stop and deliberate fault recovery. No full planning-profile
exchange with the chassis is required.

Stage 5 starts live integration. The Gazebo repository continues to contain only the
chassis plant and its execution logic. ROS task/path/obstacle handling and later Nav2
integration belong to the algorithm-side adapter. Measure physical tracking and
stopping error before treating nominal trajectory checks as plant-level evidence.
