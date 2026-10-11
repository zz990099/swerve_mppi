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
| 5A. Package/build boundaries | Complete in 0.26 | Move the standalone core into `swerve_mppi_core/`, unify installed/package names, preserve behavior, verify standalone CMake and colcon without ROS |
| 5B. ROS integration | Complete in 0.27 | Add `swerve_mppi_ros/` consuming the installed core; implement message conversion, pairing, actual chassis parameter queries, lifecycle, clock and valid command publication; add native Jazzy transport tests and ROS CI |
| 5C. Physical validation | Pending | Measure delay/response/stopping; validate straight/crab/spin, transitions, paths, obstacles and injected faults with quantitative traces |
| 6. Nav2 integration | Pending | Add a separately packaged Nav2 controller wrapper after standalone physical acceptance |

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

## Completed: stage 5A

Version 0.26 moves the existing CMake project, configuration, public/private code,
tests and offline benchmarks into `swerve_mppi_core/`. The report utility moves with
the package so a standalone copy can run all non-format tests. Root documentation,
format configuration/tooling and CI remain repository-wide. There is no root CMake
wrapper and no empty ROS package.

The manifest declares plain `cmake` build type. Project, manifest and installed
CMake config share the name `swerve_mppi_core`; callers link `swerve_mppi::core`.
Old `find_package(swerve_mppi)` aliases are removed. Installation includes the
manifest and default configuration under `share/swerve_mppi_core`. Tests consume a
relocated install prefix, and CI verifies discovery/build/test/consumption through
colcon without a ROS distribution. Algorithm behavior and chassis protocols stay
unchanged.

## Completed: stage 5B

1. Add a separate algorithm-side ROS 2 package. Keep the core and current-chassis
   adapter ROS-independent; ROS message copying, QoS, topic names and lifecycle live
   only in this wrapper.
2. Load planner and chassis parameters independently, pass only the typed shared
   chassis snapshot to the adapter compatibility check, and fail activation on a
   mismatch or missing simulation clock.
3. Buffer `ChassisState` with encoder odometry by source stamp, build fresh controller
   input, publish only valid adapter commands and withhold planning until application
   evidence is usable. Do not estimate acknowledgement from message arrival.
4. Add ROS message/lifecycle/clock/publication regressions and a colcon CI job for
   both packages. Verify that the ROS package only consumes the installed public
   core interface and declares its dependency explicitly.

The wrapper never synthesizes application acknowledgement. A publication metadata
topic and an explicit evidence service gate subsequent planning; current Gazebo
messages alone cannot close this gate. An explicit obstacle snapshot is required,
including an empty one in a verified obstacle-free scene. Runtime faults stop normal
publication and require deactivate/cleanup/configure with a fresh stopped observation.
The chassis-only recovery service retries an immutable zero request before that
reconfiguration. Clock rollback cannot be cleared by ordinary reactivation. The
unconfigured launch does not create fake task, obstacle or execution evidence.
See `ROS_INTEGRATION.md` for the wire contract and acceptance scope.

## Following: stage 5C

1. Calibrate physical command-to-motion delay, steering response, wheel acceleration,
   slip and stopping error in Gazebo. Use measurements to choose application bounds,
   history tolerances and safety margin; do not weaken nominal gates to make tests pass.
2. Validate closed-loop primitives in order: zero/timeout/recovery, straight,
   crab, spin, every mode transition and automatic realignment. Then validate paths,
   goal capture, obstacles, emergency stopping and injected delay/loss/clock faults.
3. Define quantitative acceptance thresholds and store repeatable traces. Only after
   the standalone closed loop passes should a Nav2 controller plugin translate Nav2
   path/costmap/lifecycle inputs into the same ROS-side controller service.

## Remaining sequence

Stage 5C validates physical closed loops; stage 6 adds Nav2 integration. The Gazebo
repository continues to contain only the
chassis plant and its execution logic. ROS task/path/obstacle handling and later Nav2
integration belong to the algorithm-side adapter. Measure physical tracking and
stopping error before treating nominal trajectory checks as plant-level evidence.
