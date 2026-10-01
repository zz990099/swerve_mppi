# Swerve MPPI Core

A ROS-independent C++17 hybrid controller for a four-module chassis with dual
Ackermann, spin and crab modes. It evaluates continuous controls within fixed
mode branches, predicts braking and steering alignment, and waits for measured
mode confirmation before driving after a switch.

See [ARCHITECTURE.md](ARCHITECTURE.md) for component responsibilities, execution
contracts and remaining simulation integration work.

## Build, test and install

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
find_package(swerve_mppi 0.9 CONFIG REQUIRED)
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
  frame. Lengths, angles and time use metres, radians and seconds.
- Wheel order is FL, FR, RL, RR. Core wheel speeds are **linear rolling speeds in
  m/s**. Convert joint rad/s using wheel_radius_m; do not copy raw JointState
  velocities into VehicleState::wheel_speeds.
- Steering angles are mechanical joint positions bounded by
  steering_limit_rad. Defaults use +/-90 degrees. Signed drive speeds provide
  equivalent wheel directions without crossing a steering stop.
- Timestamps must be finite, nonnegative and strictly increasing. The adapter
  owns feedback freshness checks, clock-reset handling and coordinate conversion.
- time_in_mode_s is the age of the actual confirmed mode, not the requested
  mode. mode_confirmed and mode_request_id must come from the execution layer.
  Startup uses request ID zero; subsequent IDs increase within an execution session.
- RequestMode carries Output::mode_request with a nonzero ID, explicit target
  mode and frozen steering targets. Retries preserve the complete payload. The
  executor brakes, aligns and echoes the ID before confirming. A zero velocity
  message cannot encode this action.
- Brake requests controlled stopping while retaining steering.
  Hold keeps drive stopped while applying its steering targets.
  SafeStop disables drive; its numeric targets must not be interpreted as a
  mode change or a recovery request.
- SafeStop cancels execution and latches a fault in ModeExecutor. After deliberate
  recovery, reset the executor with independently verified stopped state and reset
  the controller. Request ID high-water marks survive reset.
- NoFeasiblePlan returns Brake/Hold only after a complete stopping trajectory
  passes the shared hard validator. It reports Waiting/Blocked and retries on fresh
  input. UnsafeStoppingTrajectory, invalid input and execution faults still use
  latched SafeStop.
- Every healthy Brake, Hold and pending RequestMode also passes the stopping
  validator against fresh constraints. Normal terminal braking keeps its navigation
  status; rejection reports UnsafeStoppingTrajectory and cancels the request.
- Use TimedExecutor for queued commands. CommandEnvelope checks session, sequence,
  issue time, feedback freshness and model-period cadence. Clock resets require
  verified stopped recovery, a strictly newer session and controller.reset().

See [docs/EXECUTION_CONTRACT.md](docs/EXECUTION_CONTRACT.md) for the transport-free
ModeExecutor API, feedback mapping, cancellation and timing contract.

## Measurement tools

Enable `SWERVE_MPPI_BUILD_BENCHMARKS=ON` to build `swerve_mppi_benchmark`.
It reports solve-time percentiles, allocations and path/completion metrics as CSV.
See [docs/PERFORMANCE.md](docs/PERFORMANCE.md) for commands and measurement limits.
The optional allocation regression runs in CI; wall-clock timing is not a CI gate.

## Current status

Version 0.9 closes configurable dynamics and public safety/execution boundaries.
Signed linear/angular reversals spend braking time before accelerating in the
opposite direction. Injected validators must share the consumer's footprint,
margin and measured joint validity envelope. Model and executor bound each
module's rolling-vector residual against its declared rigid-body twist using
`drive_kinematic_tolerance_mps` (default 0.02 m/s). Switch prediction reserves at
least two post-alignment confirmation/handover ticks, even with zero configured
delay. Actual execution still requires matching measured confirmation.
Rebuild downstream consumers against 0.9; the public Config layout has changed.
See [docs/SAFETY_CONTRACT_VALIDATION.md](docs/SAFETY_CONTRACT_VALIDATION.md) for the
new contracts and current verification results.

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

**0.4 execution migration:** a Drive's `body_command` is the forward kinematics
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
