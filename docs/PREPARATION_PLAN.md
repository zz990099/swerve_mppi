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
| 4. Current interface offline adapter | Complete in 0.25 | Map rad/s to m/s, modes, request IDs, frozen receipts and conservative mode age; retain accepted ID/entry on ordinary drive/hold; reject stale outputs; compare only shared chassis parameters; test message sequences without running ROS/Gazebo |
| 5. ROS and physical integration | Next | Separate algorithm-side ROS package; validate straight/crab/spin and transitions, then paths, obstacles, stopping and delay; add Nav2 wrapper after basic closed-loop acceptance |

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

## Completed: stage 4

Version 0.25 installs current Python command/state/odometry DTOs without ROS. The
adapter pairs state with odometry, converts raw wheel angular velocity, verifies
encoder-derived twists, maps modes/faults/frozen receipts and conservatively derives
mode age from continuous confirmation. It admits only one output per latest
observation, retains the accepted request on drive/hold, records publication once
and returns caller-supplied exact or bounded application evidence.

The current chassis state does not acknowledge an MPPI command ID. Consequently the
adapter does not fabricate exact application time: transport/executor evidence stays
explicit and unknown evidence remains missing. Chassis recovery uses a separate
zero-velocity request above the observed high-water mark and retries it unchanged
until measured confirmation. Startup compares shared mechanics and command lifetime
only; planner policies remain algorithm-owned. Transport-free tests cover the current
message sequence without launching ROS or Gazebo.

## Next: stage 5

1. Add a separate algorithm-side ROS 2 package. Keep the core and current-chassis
   adapter ROS-independent; ROS message copying, QoS, topic names and lifecycle live
   only in this wrapper.
2. Load planner and chassis parameters independently, pass only the typed shared
   chassis snapshot to the adapter compatibility check, and fail activation on a
   mismatch or missing simulation clock.
3. Buffer `ChassisState` with encoder odometry by source stamp, build fresh controller
   input, publish only valid adapter commands and withhold planning until application
   evidence is usable. Do not estimate acknowledgement from message arrival.
4. Calibrate physical command-to-motion delay, steering response, wheel acceleration,
   slip and stopping error in Gazebo. Use measurements to choose application bounds,
   history tolerances and safety margin; do not weaken nominal gates to make tests pass.
5. Validate closed-loop primitives in order: zero/timeout/recovery, straight,
   crab, spin, every mode transition and automatic realignment. Then validate paths,
   goal capture, obstacles, emergency stopping and injected delay/loss/clock faults.
6. Define quantitative acceptance thresholds and store repeatable traces. Only after
   the standalone closed loop passes should a Nav2 controller plugin translate Nav2
   path/costmap/lifecycle inputs into the same ROS-side controller service.

## Remaining sequence

Stage 5 starts live integration. The Gazebo repository continues to contain only the
chassis plant and its execution logic. ROS task/path/obstacle handling and later Nav2
integration belong to the algorithm-side adapter. Measure physical tracking and
stopping error before treating nominal trajectory checks as plant-level evidence.
