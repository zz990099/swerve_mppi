# Pre-integration acceptance matrix (0.21.4)

This checklist separates completed portable contracts from unimplemented transport
and unproven physical behavior. Green core tests are not an acceptance report for
the unchanged companion simulator. The normative owner obligations are in
ADAPTER_READINESS.md and MODEL_FEEDBACK.md.

| Requirement | Portable implementation/evidence | Remaining integration proof |
| --- | --- | --- |
| Complete resolved configuration and numeric types | parse_resolved_config_profile; every missing field and malformed/duplicate/unknown profile rejected in adapter_regressions | Exchange actual resolved participants' configurations; reject incomplete wire messages |
| Shared model/admission/stopping configuration | Exact execution_config_mismatches/require_adapter_compatible; planning-only differences allowed | Include actual plant geometry/limits and recreate consumers after changes |
| Clock domain/reset incarnation and frames | AdapterContract and check_snapshot_contract; frame/domain/epoch and original ns mismatch regressions | Bind real clock sources and TF provenance, command clock epoch and exclusive session authority |
| Current coherent ingress | FeedbackAdapter + metadata/payload/current-callback binding in fixture; adjacent ns and stale whole packet rejected | Actual collection/time alignment of pose, joints, mode and constraints; transport delay/uncertainty |
| Independent motion policy | MotionObserver; missing/correlated/noisy/bounded/slipping residuals fail independent nominal owner admission | Independent body/pose source and justified deterministic bounds; robust model needed to accept nonzero uncertainty |
| Joint units, names and module order | FeedbackAdapter; missing/duplicate/malformed joint regressions; FL/FR/RL/RR mapping | Actual names, signs, axes, calibration and wheel-radius conversion |
| Command identity and lifetime | TimingGuard/TimedExecutor and adapter fixture; missing, stale/future/expired, wrong session, replay and source task rejection | Complete wire decoding; preserve source/task identity through queues and restarts |
| Mode ownership and immutable accepted receipt | Existing execution/chassis/motion suites plus rejected-request fixture; no request committed on ingress/task rejection | Actual status stream/confirmation, exact receipt echo and one supervisor |
| Immediate external-fault revocation | ProfileRunner::cancel; installed consumer and adapter fixture | Serialize cancellation with final publication; discard queued/copied samples; invoke actual stop channel |
| Non-driving checked rejection fallback | TimedExecutor + ProfileRunner; task mismatch retains healthy separately checked fallback | Publish execution diagnostics, never obsolete planner success or obsolete Drive |
| Profile interval and wall-clock failure | ProfileRunner; wall pause/rollback, application rollback and interval expiry regressions | Actual high-rate scheduling under simulator/ROS load and independent endpoint watchdog |
| Startup/fault recovery | Fixture continuous stop dwell, gap/repeated/moving rejection, endpoint/drain acknowledgements, newer session and old-session replay rejection | Physical stopped acknowledgement bound to recovery/session; drain actual queues; coordinated endpoint arm/reset |
| Target runtime budget | Nine-case matrix with raw work/timing/build/config evidence and strict runner available | Strict acceptance on intended CPU with simulator/transport contention; missed sample/deadline instrumentation |
| Prediction versus independent plant | 720-sample synthetic lag/noise/slip/delay probe and independent encoder/profile behavior fixtures | Identify actuator/braking/steering response, tire slip, minimum braking and tracking envelopes |
| Collision/stopping safety | Nominal swept interval and complete-stop validation remain strict | Robust initial/propagated pose/twist/joint uncertainty and physical braking envelopes before accepting real collision guarantees |

## Test mapping

`adapter_regressions` exercises complete startup exchange, all frame/clock record
positions, exact timestamp binding, already-active Drive revocation, missing and
correlated independent observations, bounded noise/residual rejection, malformed
joints, command lifetime/replay/task fallback, watchdog/clock failure, sustained
stop recovery and queue/endpoint acknowledgement requirements. `installed_consumer`
compiles/links the new contract, resolved-profile and cancel APIs from a clean install.
The Debug CI job additionally runs adapter/configuration regressions with UBSan.
Existing configuration, timing, chassis, motion, actuation, admission and behavior
suites remain required. CMake builds the library; no direct compiler entry exists.

No test in this phase proves total process death detection, thread scheduling,
physical emergency braking or endpoint acknowledgement truth. Those belong to an
independent endpoint/plant and the later adapter's acceptance suite. The fixture's
stop_requests counter and acknowledgement booleans are abstract test inputs, not
safety evidence from an actuator.

## Subsequent work after this preparation request

1. Review the companion's actual resolved profiles, frame/clock namespaces,
   transport fields, mode receipt and exclusive endpoint against this contract.
   Decide a nominal stepped validation policy before editing its ROS adapter.
2. Implement explicit startup exchange, ingress binding, command lifetime,
   cancellation/endpoint watchdog and supervised recovery in that adapter.
   Do not silently relax model admission to make noisy feedback run.
3. Run stepped nominal closed loops, every directed mode transition, braking,
   cancellation, stale/replayed packets, shutdown/process loss and recovery with
   recorded contract/configuration and source/build evidence.
4. Identify independent plant response and uncertainty; implement robust prediction,
   stopping and transition bounds if noisy/slipping physical acceptance is needed.
5. Prove runtime deadlines on the target CPU under realistic contention. A Nav2
   plugin/lifecycle wrapper follows the same ownership contracts when needed.

These are follow-up items, not work performed or authorized in this preparation
series. The core architecture is retained; no wholesale rewrite or new ROS/Gazebo
wiring is required to complete this stage. Preparation readiness is established
for contract review and offline nominal testing, not independent physical deployment.
