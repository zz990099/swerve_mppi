# Swerve MPPI Core

A ROS-independent C++17 hybrid controller for a four-module chassis with dual
Ackermann, spin and crab modes. It evaluates continuous controls within fixed
mode branches, predicts braking and steering alignment, and waits for measured
mode confirmation before driving after a switch.

See [ARCHITECTURE.md](ARCHITECTURE.md) for component responsibilities, execution
contracts and remaining simulation integration work.
See [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) for module directories, the 0.19
header migration and the official ROS Rolling formatting/checking workflow.

## Build, test and install

Version 0.21.0 adds shared fail-closed segment geometry and exact immutable spatial
indices reused across every branch of a planning call. Public scoring and execution
still validate fresh inputs independently. Live-budget probes now cover the ROS
60 ms budget and up to 4096 path points without truncating obstacles or paths.
See [docs/READINESS_VALIDATION.md](docs/READINESS_VALIDATION.md) for acceptance scope.

Version 0.20.3 adds configurable live-budget measurement and an opt-in target-host
timeout gate to `swerve_mppi_integration_budget`; control semantics remain 0.20.2.
The companion `swerve_gazebo_sim` repository now supplies the ROS planning node,
explicit execution controller and physical closed-loop regressions on
Humble/Fortress and Jazzy/Harmonic. Those results apply to the bundled nominal
plant; independent localization, slip uncertainty and hardware calibration remain
integration work. Older release notes below describe historical milestones.

Version 0.20.2 enters checked terminal translation for a GoalOnly task that starts
in Spin, preventing finite-horizon switch costs from retaining a mode that cannot
advance position. Mode dwell, measured stopping, full transition and complete-stop
validation still apply. See [docs/SPIN_TRANSLATION_VALIDATION.md](docs/SPIN_TRANSLATION_VALIDATION.md).

Version 0.20.1 rejects unrepresentable derived path geometry with `InvalidPath`,
absent command authorization and finite diagnostics. Normalized projection and
turn calculations preserve representable geometry without overflowing squared
lengths or dot/cross products. See [docs/PATH_NUMERICAL_VALIDATION.md](docs/PATH_NUMERICAL_VALIDATION.md).

Version 0.20 added executor-owned accepted entry geometry to mode feedback, so
pending-request safety checks follow the actual committed steering interval.
It also makes nonzero entry geometry independent of velocity amplitude and avoids
finite-yaw subtraction overflow. See [docs/ENTRY_FEEDBACK_VALIDATION.md](docs/ENTRY_FEEDBACK_VALIDATION.md)
for regressions and validation. Downstream consumers must rebuild and forward the
new feedback field; the actionless chassis command and module include paths remain.
The earlier Drive interval certification is documented in
[docs/BOUNDARY_REVIEW_VALIDATION.md](docs/BOUNDARY_REVIEW_VALIDATION.md).

Requirements: CMake 3.20 or newer and a C++17 compiler. CMake is the only supported
build entry point; no workspace activation script or ROS environment is required.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install-local"
```

Set SWERVE_MPPI_BUILD_TESTS=OFF for a library-only build.
BUILD_SHARED_LIBS=ON selects a shared library on supported toolchains.
CTest runs the original behavior checks, model, optimizer and execution
and navigation regressions, default-noise multi-seed behavior checks, and an independent consumer that finds and links the installed CMake package.
CI builds Debug and Release configurations through CMake.

Downstream CMake projects use the exported target:

```cmake
find_package(swerve_mppi 0.21 CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE swerve_mppi::core)
```

Add the install prefix to CMAKE_PREFIX_PATH.

## Controller contract

Call Controller::compute(input) once per Config::dt_s using fresh, consistent
measured vehicle state and a nonempty ordered task path. Keep the same path
and `path_id` on subsequent ticks; the core manages progress and local pruning. The current warm-start sequence
advances by one model step per accepted drive action; arbitrary controller/model
period ratios are not supported yet.

- Pose and path coordinates share one world frame. Velocities are in the robot
  frame. Lengths, angles and time use metres, radians and seconds. Finite unwrapped
  yaw is reduced periodically before angular differences/transforms. Nonfinite
  derived goal errors withhold authorization with InvalidInput.
- Wheel order is FL, FR, RL, RR. Core wheel speeds are **linear rolling speeds in
  m/s**. Convert joint rad/s using wheel_radius_m; do not copy raw JointState
  velocities into VehicleState::wheel_speeds.
- Steering angles are mechanical joint positions bounded by
  steering_limit_rad. Defaults use +/-90 degrees. Signed drive speeds provide
  equivalent wheel directions without crossing a steering stop.
- Timestamps must be finite, nonnegative and strictly increasing. The adapter
  owns feedback freshness checks, clock-reset handling and coordinate conversion.
- time_in_mode_s is the age of the actual confirmed mode, not the requested
  mode. mode_confirmed, mode_request_id and accepted_mode_request must come from
  the execution layer.
  Startup uses request ID zero; subsequent IDs increase within an execution session.
- `Output::command` is an optional `ChassisCommand`: explicit mode, body-frame
  `target_velocity` (`vx`, `vy`, `wz`) and optional `mode_request`. The planning
  output has no action or joint targets. See [docs/CHASSIS_COMMAND.md](docs/CHASSIS_COMMAND.md).
- A valid zero target requests normal braking/holding and retains the actual mode.
  An absent command cancels execution and latches a fault; never reuse old velocity.
  Independently stop/verify the plant before resetting execution and Controller.
- A mode request carries a nonzero ID, target mode and frozen body `entry_velocity`.
  Its target_velocity is exactly zero. Entry intent specifies alignment geometry,
  never permission to drive. Retries preserve the entire request; execution confirms
  the matching ID/mode using stopped body/joints and measured steering. Execution
  echoes its frozen mechanical entry in accepted_mode_request; prediction uses that
  exact geometry after acceptance, and final handover requires measured alignment.
- Target velocity is the nominal intent, rather than FK of rate-limited joint
  endpoints. The execution layer compiles it at the current snapshot with the same
  DriveModel used in prediction. Large steering changes still require stop/alignment.
- NoFeasiblePlan returns a valid zero command only after a complete stopping
  trajectory passes the shared hard validator, with Waiting/Blocked diagnostics.
  UnsafeStoppingTrajectory, invalid input and execution faults withhold authorization.
- Every driving intent passes a first-command plus complete-stop check before
  publication or warm-start acceptance. `stopping_horizon_steps` bounds this work
  independently of the MPPI horizon. Normal stops and pending requests also pass
  fresh stopping/steering checks. Joint feedback remains required input.
- Use TimedExecutor for queued commands. Pass the current ControllerInput, not just
  VehicleState. It rechecks the exact joint interval and full stopping continuation
  against current obstacles and injected hard constraints before committing execution.
- Bind every envelope with CommandTask::capture(planning_input). A changed path ID,
  full ordered path geometry or heading policy rejects an old queued command before
  mode preview. Missing task metadata also rejects it; TaskMismatch returns checked
  stopping when feasible, otherwise SafeStop latches.
- CommandEnvelope requires source_stamp_s, execute_at_s and valid_until_s in addition
  to session, sequence and issue time. A fresh issue time cannot disguise an old
  planning observation. Execution-start state must be aligned to now_s; merely recent
  raw feedback is insufficient. TimingGuard checks transport age; the adapter owns
  time alignment and any model/measurement uncertainty.
- A rejected well-formed command returns checked braking with CommandRejected only
  if a complete stop remains feasible. Unsafe stopping latches SafeStop. Timing or
  protocol faults also latch SafeStop. Clock recovery requires a verified stopped
  state, a strictly newer session and controller.reset().
- Healthy TimedExecutionResult::actuation is a checked ActuationPlan. Sample it at
  the actuator rate; Drive ramps both joint arrays over the whole model tick, while
  Hold/RequestMode finish proportional braking before steering. Expired samples
  return no target. SafeStop has no certified normal-braking profile.

See [docs/EXECUTION_CONTRACT.md](docs/EXECUTION_CONTRACT.md) for the transport-free
guarded execution cycle, feedback mapping, cancellation and timing contract.

## Measurement tools

Enable `SWERVE_MPPI_BUILD_BENCHMARKS=ON` to build `swerve_mppi_benchmark`.
It reports solve-time percentiles, allocations and path/completion metrics as CSV.
See [docs/PERFORMANCE.md](docs/PERFORMANCE.md) for commands and measurement limits.
The optional allocation regression runs in CI; wall-clock timing is not a CI gate.

## Current status

Version 0.19 organizes public headers, implementations and tests into seven functional
modules, removes model-to-executor header coupling for shared value types and narrows
the Controller header boundary. Formatting uses the unchanged official Rolling
ament_clang_format configuration with pinned developer tools and a CI/CTest gate.
Downstream includes must add their module directory; see
[docs/DEVELOPMENT.md](docs/DEVELOPMENT.md). Algorithms and chassis command semantics
retain the 0.18 behavior.

Version 0.18 replaces the public joint/action output with optional chassis velocity
and explicit mode requests. Controller hides its prediction implementation;
ChassisExecutor provides a synchronous body-command reference supervisor and
TimedExecutor compiles body targets before guarded execution. Rebuild consumers
and migrate field access using [docs/CHASSIS_COMMAND.md](docs/CHASSIS_COMMAND.md).
All behavior, profile, benchmark and installed-consumer paths use the new interface.
The following release notes describe older interfaces where indicated.

Version 0.17.1 adds exact integer-nanosecond snapshot admission for ROS ingress
and permits only floating conversion roundoff in the portable seconds API.

Version 0.17 adds a named, timestamp-strict FeedbackAdapter, actual ProfileRunner
closed-loop tests for 14 scenarios and five seeds, a production-budget pipeline
probe, and PIC builds for downstream ROS plugins. Gazebo package 0.2 supplies an
exclusive independent guarded joint endpoint. See
[docs/SIMULATION_PREPARATION.md](docs/SIMULATION_PREPARATION.md) for scheduling,
wire ownership, recovery and the remaining physical acceptance work.
A complete ROS MPPI planning node and measured Gazebo profile tracking remain next-stage work.


Version 0.16 separates configurable feedback diagnostics from nominal model
admission. `check_feedback` retains inclusive 0.05 m/s / 0.10 rad/s defaults;
`check_model_feedback` requires body/encoder agreement within numerical precision
(1e-9), so even smaller unmodelled residual motion cannot certify a normal stop.
Ordinary failed planning, deadline expiry, replan and alignment discard warm starts
without reseeding noise; explicit Controller/Optimizer reset reproduces the search.
Injected validators require matching wheelbase and track as well as safety settings.
ProfileRunner accepts checked non-driving pending-alignment rejection fallbacks,
retaining the frozen request and deadline. Drive rejection remains fail-closed.
Bounded inputs and shared cooperative planning budgets remain enabled. Rebuild
consumers against 0.16 to use the new admission and warm-start APIs.
See [docs/ADMISSION_VALIDATION.md](docs/ADMISSION_VALIDATION.md) for limits and tests.
The Gazebo repository has a protected external joint command mode; a complete ROS planning adapter,
measurement time alignment and physical braking/slip/latency calibration remain
integration work. Core regression success does not certify the physical plant.

Version 0.14.1 validates the actual Brake/Hold/RequestMode interval and its full
stopping tail before Controller publication, including alignment during pending
mode retries. A stationary brake alone cannot authorize steering against newly
changed hard constraints. Planning and guarded execution share ActuationModel's
non-driving profile; nominal checks preserve measured mode feedback and never
synthesize confirmation. Use the guarded cycle in the execution contract for
integration, and sample its checked ActuationPlan at the actuator rate.
Rebuild consumers against 0.14.1 because Controller's private layout changed.

Version 0.14 adds execution-time safety validation and a checked actuator profile.
TimedExecutor now requires a current ControllerInput and complete scheduling/source
metadata plus the originating task snapshot; the old state-only update API is removed.
It validates the actual joint
endpoint interval, rather than reconstructing a new control from body_command.
Unsafe delayed/context-changed commands fall back only to a separately validated
complete stop. Protocol preview is transactional, so rejecting a new mode request
cannot consume its ID or commit an unexecuted transition. ActuationPlan exposes
bounded high-rate target samples and its swept-motion endpoint. Rebuild consumers
against 0.14. See [docs/EXECUTION_CONTRACT.md](docs/EXECUTION_CONTRACT.md) and
[docs/EXECUTION_SAFETY_VALIDATION.md](docs/EXECUTION_SAFETY_VALIDATION.md).


Version 0.13 fixes marginal noise weighting across inactive alignment ticks and
rejects trajectories whose initial pose differs from current vehicle feedback.
NoiseGenerator::correction accepts an optional Branch argument; callers sampling
mode switches must pass it. Rebuild consumers against 0.13 because its symbol
signature changed. See docs/CORRELATION_ANCHOR_VALIDATION.md for verification.

Version 0.12 closes the residual-motion and full-period execution review findings.
Hold/RequestMode finish proportional braking before steering, including rolling
speeds below handover thresholds. Stable Drive requires freshly confirmed,
matching measured mode feedback. Absolute body speed limits apply throughout the
Drive interpolation, with bounded certification that fails closed. Duplicate
path points preserve Ackermann curvature speed limits. Rebuild against 0.12 and
update actuator adapters to the phased stopping/alignment contract; public layouts
are unchanged from 0.11. See
[docs/ACTUATION_BOUNDARY_VALIDATION.md](docs/ACTUATION_BOUNDARY_VALIDATION.md)
for regressions and current measurement evidence.


Version 0.11 made Drive prediction and execution use affine wheel-speed and
steering interpolation over the entire control period. The model integrates the
encoder velocity field with analytic yaw and conservative translation/sweep
bounds; the executor checks absolute mode speed limits and pointwise body rates
separately from transient mode-projection tolerance. Measured overspeed recovers
through the checked Brake path. A rejected first-Drive stopping continuation can
retry bounded reductions of the same intent, preserving entry geometry and
clearing the warm start after a successful reduction. Tight local Ackermann
curves also anticipate the yaw-rate speed budget.
The 0.11 release changed Config and Output layouts.
See [docs/DRIVE_EXECUTION_VALIDATION.md](docs/DRIVE_EXECUTION_VALIDATION.md) for the
contract, independent oracles and verification evidence. The
[0.10 stopping review](docs/STOPPING_REVIEW_VALIDATION.md) remains historical evidence.

Version 0.9 closes configurable dynamics and public safety/execution boundaries.
Signed linear/angular reversals spend braking time before accelerating in the
opposite direction. Injected validators must share the consumer's footprint,
margin and measured joint validity envelope. Model and executor bound each
module's rolling-vector residual against its declared rigid-body twist using
`drive_kinematic_tolerance_mps` (default 0.02 m/s). Switch prediction reserves at
least two post-alignment confirmation/handover ticks, even with zero configured
delay. Actual execution still requires matching measured confirmation.
The 0.9 release changed the public Config layout.
See [docs/SAFETY_CONTRACT_VALIDATION.md](docs/SAFETY_CONTRACT_VALIDATION.md) for the
0.9 contracts and historical verification results.

Version 0.8 gives navigation an explicit effective target and terminal eligibility.
Uncaptured corners own their slowdown/capture target, even near a closed-loop or
foldback endpoint. Goal acquisition and terminal capture require that eligibility.
All controlled stops share one output check, including terminal early returns,
completed-task external motion and unconfirmed mode requests. Stopping predictions
preserve measured mode feedback and never synthesize confirmation. New closed loops
cover short foldbacks, short squares, near-terminal corners and a close obstacle
corridor. Those public navigation/rollout additions required rebuilding 0.8 consumers.
See [docs/CORE_REVIEW_VALIDATION.md](docs/CORE_REVIEW_VALIDATION.md) for the historical
0.8 regression results and committed Release measurement matrix.

Version 0.7 fixes segment-order matching on short loops/crossings/foldbacks,
unifies tracking/corner/terminal alignment and preserves switch-entry intent through
the first predicted and executed Drive. A shared TrajectoryValidator separates hard
constraints from scoring. Capture/alignment checks the next Drive and stopping
continuation instead of extrapolating a constant command past a nearby goal.
TimingGuard and TimedExecutor provide transport-free timing checks; sparse feedback
cannot establish continuous stopped completion. See
[docs/PRE_SIMULATION_VALIDATION.md](docs/PRE_SIMULATION_VALIDATION.md) for scope and validation.

The default dimensions, joint rates and body limits are standalone configuration
assumptions. Verify them against the target chassis and independently identify
braking/steering response before integration. This repository contains the core
and its encoder fixture; the defaults have no bundled simulator calibration evidence.
Components own their configuration, so temporaries and copied controllers cannot
leave dangling configuration references.

Version 0.6 reuses optimizer/rollout storage, retains safety-check models and
limits path geometry scans to their forward windows. Path scoring computes the
nearest-segment distance/yaw once per pose. `Output` now explains the computation
policy, failure reason and work across all planning branches. Returned solutions
still own their data; controller/optimizer objects must be called serially.

Version 0.5 added path progress, bounded local references, sharp-corner capture,
terminal slowdown, pose alignment and measured-stop completion. It supports
reverse waypoint order and an explicit body-heading policy. New geometry,
heading policy or `path_id` resets task progress; completed tasks remain stopped.
See [docs/NAVIGATION.md](docs/NAVIGATION.md) for lifecycle, tuning and migration,
and [docs/VALIDATION.md](docs/VALIDATION.md) for the 45 seeded completion runs.

Version 0.4 established default-noise core behavior. Small steering changes in a
stable mode now advance steering and drive together, bounded by steering rate,
wheel acceleration and body acceleration. Larger changes retain stop/align/drive
and freeze the entry intent until the first Drive or a timeout. Mode switches
still require measured confirmation and a stopped handover. Gaussian proposals
now have configurable temporal correlation (`noise_correlation`, default 0.85;
zero restores independent noise).

`drive_steering_limit_rad` (default 0.20) bounds the requested joint change that
may proceed while driving. Set it to `steering_tolerance_rad` for a conservative
stop/align policy. This parameter represents an actuator capability assumption
and must be calibrated before hardware deployment. Mechanical hard stops always
use direct joint distances; equivalent wheel directions never bypass them.

**Historical 0.4 execution migration (joint layer):** a Drive's `body_command` is the forward kinematics
of its **wheel-speed targets and steering targets**, not the old measured angles.
Drive steering targets are the bounded next joint step, not an unrestricted final
angle. Update custom execution supervisors accordingly; do not reconstruct wheel
commands from body twist and discard these joint targets. Nonfinite body commands
and excessive moving steering steps are rejected by ModeExecutor.

This is a core research prototype. It has no ROS node or Nav2 plugin, and has not
been validated in a combined Gazebo closed loop. Obstacles and the footprint are
circles, and each horizon allows one mode change. Footprint/costmap queries,
actuator-delay calibration, solve-time profiling and transport adapters remain
future work. Path tracking and completion are implemented for ordered task paths;
localization jumps and unrestricted global-path reacquisition are not supported. The typed
mode contract and standalone execution supervisor are implemented and tested.
These core tests do not establish agreement with physical or Gazebo dynamics.
