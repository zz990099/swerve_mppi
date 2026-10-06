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
| 1: configuration foundation (complete, 0.21.1) | Complete typed registry, portable profiles, execution compatibility API, resolved benchmark configuration, current-status documentation | Configuration and installed-consumer regressions; existing core suite; no ROS/Gazebo code changes |
| 2: timing and workload diagnostics (complete, 0.21.2) | Separate controller and full-pipeline costs, per-call work/status traces, nine-case matrix with configuration/build/source/host evidence | Recording versus strict acceptance; deterministic accounting tests and CLI/manifest cross-checks; target-host procedure; no library callback instrumentation |
| 3: model and feedback preparation | Characterize the current exact encoder/body admission assumption; design explicit bounded uncertainty and fault contracts; add independent offline lag/noise/slip probes | Fault/stop/mode invariants remain fail-closed; model prediction error is measured against independent observations; no silent tolerance widening |
| 4: adapter readiness | Specify resolved-config exchange, coherent state frames/timestamps, command lifetime, watchdog and recovery ownership; core contract fixtures and checklist | Reviewable pre-integration interface and offline acceptance matrix; no new simulator/ROS wiring |

Stage 1 adds APIs and measurement support. It does not make configuration agreement
automatic across processes, relax the 1e-9 nominal model agreement gate, prove the
60 ms target-host budget, or address live executor scheduling stalls.

Stage 2 preserves the bounded input policy and all safety gates. Controller timing
includes path preparation, geometry, optimization and safety work; it is not an
optimizer-only measurement. Changing input density and sample/horizon profiles
supports scaling comparisons. Process CPU gaps are hints rather than proof of
scheduling causes; internal critic/rollout profiling and live executor stall
diagnosis remain separate investigations. CI records timing; strict runtime
acceptance belongs on the intended CPU. See BUDGET_DIAGNOSTICS.md.

Stage 3 is the next implementation batch and requires a concrete uncertainty policy before changing model admission.
Measured body motion, encoder kinematics and stopping certification must have
separate responsibilities. Offline probes can reveal unsafe assumptions without
claiming physical calibration. Stage 4 packages those decisions for later use.

Actual ROS/Gazebo migration, new physical tests, independent localization and a
Nav2 controller plugin are subsequent work outside this preparation request.
Stop after stage 4 and report readiness and remaining integration risks.
