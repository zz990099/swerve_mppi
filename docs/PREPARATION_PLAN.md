# Preparation before further simulation integration

This series changes the standalone algorithm core and offline verification only.
The companion repository already contains a ROS planner, execution controller and
nominal Gazebo regressions. This preparation series neither extends that
integration nor runs new physical acceptance. Historical results are not evidence
for newly introduced core behavior.

The existing module separation remains useful: navigation prepares the local
task, planning evaluates mode branches and continuous controls, models predict
motion, safety validates continuations, and execution owns guarded actuation.
Preserve these boundaries and incrementally improve their contracts. A wholesale
rewrite would discard tested mode/stop semantics without resolving the current
configuration, timing and uncertainty gaps. Nav2 MPPI can guide optimizer/noise/
critic ownership and lifecycle boundaries; its body-twist interface does not
replace the swerve mode handshake or the execution supervisor.

| Stage | Deliverable | Acceptance boundary |
| --- | --- | --- |
| 1: configuration foundation | Complete typed registry, portable profiles, execution compatibility API, resolved benchmark configuration, current-status documentation | Configuration and installed-consumer regressions; existing core suite; no ROS/Gazebo code changes |
| 2: timing and workload diagnostics | Separate planning and full-pipeline costs, record workload/branch/evaluation counts with configuration, reproducible offline workload matrix | Functional failures distinguished from cooperative timeouts and pipeline overruns; tests of accounting; target-host procedure |
| 3: model and feedback preparation | Characterize the current exact encoder/body admission assumption; design explicit bounded uncertainty and fault contracts; add independent offline lag/noise/slip probes | Fault/stop/mode invariants remain fail-closed; model prediction error is measured against independent observations; no silent tolerance widening |
| 4: adapter readiness | Specify resolved-config exchange, coherent state frames/timestamps, command lifetime, watchdog and recovery ownership; core contract fixtures and checklist | Reviewable pre-integration interface and offline acceptance matrix; no new simulator/ROS wiring |

Stage 1 adds APIs and measurement support. It does not make configuration agreement
automatic across processes, relax the 1e-9 nominal model agreement gate, prove the
60 ms target-host budget, or address live executor scheduling stalls.

Stage 2 is the next implementation batch. It should retain the existing bounded
input policy and all safety gates, and expose enough evidence to decide whether
cost traversal, branch count, sample count or external scheduling dominates. CI
records timing; strict runtime acceptance belongs on the intended CPU.

Stage 3 requires a concrete uncertainty policy before changing model admission.
Measured body motion, encoder kinematics and stopping certification must have
separate responsibilities. Offline probes can reveal unsafe assumptions without
claiming physical calibration. Stage 4 packages those decisions for later use.

Actual ROS/Gazebo migration, new physical tests, independent localization and a
Nav2 controller plugin are subsequent work outside this preparation request.
Stop after stage 4 and report readiness and remaining integration risks.
