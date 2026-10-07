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
| 3: model and feedback preparation (complete, 0.21.3) | Independent observation/bounded residual API, explicit nominal-only fault/stop policy, 720-sample independent lag/noise/slip/delay matrix | Fault/stop/mode invariants remain fail-closed; model prediction error is measured against independent observations; no silent tolerance widening |
| 4: adapter readiness (complete, 0.21.4) | Complete peer profile parser, immutable adapter/frame/clock contract, explicit profile revocation, serialized offline owner fixture and acceptance checklist | Reviewable pre-integration interface and offline acceptance matrix; no new simulator/ROS wiring |

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

Stage 3 preserves nominal model admission and supplies a stateless diagnostic
sidecar, not an automatically enforced independent-motion gate. The offline plant
exposes prediction error despite encoder/body agreement and false encoder-only
stop evidence during chassis coasting. Noisy bounded residuals remain inadmissible;
accepting them requires reviewed robust stopping/transition envelopes and physical
identification. See MODEL_FEEDBACK.md for the exact uncertainty and fault policy.

Stage 4 supplies startup contract and original frame/clock/stamp checks plus
ProfileRunner::cancel for immediate external-fault profile revocation. A serialized
offline owner fixture exercises binding to an independent current callback clock,
strict independent motion, complete command/task lifetime, continuous stopped
recovery and acknowledged drain/endpoint stop. The contract APIs do not automatically
arm or retrofit external callers. See ADAPTER_READINESS.md and ADAPTER_ACCEPTANCE.md.

Preparation stages 1..4 are complete. A separately requested follow-up now audits
and prepares the companion without connecting it. Stage 5 (complete, 0.21.5) adds
the pinned source audit, configuration provenance and offline candidate preflight.
See SIMULATION_MIGRATION_AUDIT.md for concrete evidence and migration obligations.
The companion remains unchanged, and prepared candidates are not runtime exports
or physical acceptance.

Next preparation batches keep the same no-integration boundary: versioned wire
records and bounded decoding fixtures, then serialized revocation/recovery and
acknowledgement fixtures, then independent measurement/calibration specifications.
Actual ROS/Gazebo migration and physical tests require a later integration request.
Independent noisy/slipping motion, robust braking guarantees and target-host
scheduling acceptance remain open. A Nav2 plugin remains subsequent work.
