# Staged implementation plan

Target boundary: a ROS-independent planner returns body velocity and explicit mode
requests; the independent Gazebo chassis executes those requests. No old execution
or transport interface will receive compatibility wrappers. Complete one stage per
iteration, test it, and push directly to main before starting the next stage.

| Stage | Status | Deliverable and acceptance |
| --- | --- | --- |
| 1. Planning/execution ownership | Complete in 0.22 | Remove production executors, profile sampling, session/sequence and peer adapter contracts; privatize prediction commands; remove obsolete companion preparation tools; preserve planning regressions, nominal closed-loop behavior and installed-consumer checks |
| 2. Current chassis prediction | Next | Match Python speed/steering limits, normal zero, brake-align-confirm transitions and same-mode realignment; preserve mode dwell/hysteresis and measured acknowledgement; add offline parity tests against current chassis semantics |
| 3. Observation, timing and faults | Pending | One timestamp representation; preserve observation time separately from computation and command expiry; define freshness, pairing, reset and rejection; separate numerical consistency from measured uncertainty; review control/model periods and warm-start shifts |
| 4. Current interface offline adapter | Pending | Map rad/s to m/s, modes, request IDs, frozen receipts and mode age; retain accepted ID/entry on ordinary drive/hold; reject stale outputs; compare only shared chassis parameters; test message sequences without running ROS/Gazebo |
| 5. ROS and physical integration | Pending | Separate algorithm-side ROS package; validate straight/crab/spin and transitions, then paths, obstacles, stopping and delay; add Nav2 wrapper after basic closed-loop acceptance |

## Next: stage 2

Use the current Python chassis contract as the behavior reference. Compare the core's
limits and normal stopping law with that implementation before choosing model
parameters. The current affine joint interpolation and proportional braking are
nominal assumptions, not commands the chassis has agreed to execute.

Predict normal zero holding steering, explicit brake-align-confirm mode changes,
and automatic realignment within a mode. Use the chassis's accepted steering receipt
after request acceptance. Never infer actual confirmation from elapsed predicted time.
Keep switch cost, hysteresis, minimum mode age and immutable pending requests.

Acceptance covers all six directed mode changes, signed reverse motion, ordinary
zero, same-mode realignment and transition timing/target limits. Retain separate
model-error measurements: matching deterministic command semantics does not establish
Gazebo servo/contact or physical stopping accuracy. Do not add ROS/Gazebo wiring in
this stage.

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
