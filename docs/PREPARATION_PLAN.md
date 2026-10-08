# Staged implementation plan

Target boundary: a ROS-independent planner returns body velocity and explicit mode
requests; the independent Gazebo chassis executes those requests. No old execution
or transport interface will receive compatibility wrappers. Complete one stage per
iteration, test it, and push directly to main before starting the next stage.

| Stage | Status | Deliverable and acceptance |
| --- | --- | --- |
| 1. Planning/execution ownership | Complete in 0.22 | Remove production executors, profile sampling, session/sequence and peer adapter contracts; privatize prediction commands; remove obsolete companion preparation tools; preserve planning regressions, nominal closed-loop behavior and installed-consumer checks |
| 2. Current chassis prediction | Complete in 0.23 | Match Python speed/steering limits, normal zero, brake-align-confirm transitions and same-mode realignment; preserve mode dwell/hysteresis and measured acknowledgement; add offline parity tests against current chassis semantics |
| 3. Observation, timing and faults | Next | One timestamp representation; preserve observation time separately from computation and command expiry; define freshness, pairing, reset and rejection; separate numerical consistency from measured uncertainty; review control/model periods and warm-start shifts |
| 4. Current interface offline adapter | Pending | Map rad/s to m/s, modes, request IDs, frozen receipts and mode age; retain accepted ID/entry on ordinary drive/hold; reject stale outputs; compare only shared chassis parameters; test message sequences without running ROS/Gazebo |
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

## Next: stage 3

1. Replace seconds/nanoseconds dual ingress with one source timestamp representation.
   Keep source observation time, computation time, publication time and expiry distinct.
2. Define observation pairing, maximum age/skew, command lifetime and stale-result
   rejection. Do not require exact application-time equality or relabel old data.
3. Separate numerical internal consistency from measured uncertainty and physical
   residual admission. Define bounds for stopping/motion rather than only relaxing 1e-9.
4. Reconcile command prediction history with asynchronous measured joints/mode phases.
   Hidden limited velocity is not observable in the current chassis state; explicitly
   handle unknown startup state and uncertain command application.
5. Define clock reset, feedback/compute/transition fault latching and deliberate recovery.
   Clear planning history and warm starts according to those rules.
6. Separate planner call period, rollout period and chassis update period; shift warm
   starts by elapsed intervals and bound timing/work. Test delay, skew, duplicate/out-of-
   order samples, expiry, discontinuity and recovery before adapter work.

## Remaining sequence

Stage 3 removes the exact observation/application-time assumption and the dual
seconds/nanoseconds ingress, and defines bounded observation-age and failure handling.
Do not make old data current by relabelling timestamps. Physical uncertainty requires
an explicit admission policy; merely loosening 1e-9 gates is insufficient.

Stage 4 adds only the current command/state mapping and offline sequence tests. It
must preserve request identity, accepted geometry and source-result lifetime while
supporting normal stop and deliberate fault recovery. No full planning-profile
exchange with the chassis is required.

Stage 5 starts live integration. The Gazebo repository continues to contain only the
chassis plant and its execution logic. ROS task/path/obstacle handling and later Nav2
integration belong to the algorithm-side adapter. Measure physical tracking and
stopping error before treating nominal trajectory checks as plant-level evidence.
