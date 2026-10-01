# Core architecture

The library builds as swerve_mppi::core using CMake. Its public headers contain
no ROS, Nav2, Gazebo or pluginlib dependencies. ROS validation nodes and a future
Nav2 controller plugin belong in separate adapter packages.

## Components

| Component | Responsibility |
| --- | --- |
| Controller | Validate input, coordinate path/goal state, continue committed transitions and emit one action. |
| PathManager | Monotonic arc progress, bounded matching, local reference and corner capture. |
| GoalManager | Pose acquisition, measured-stop settling, completion latch and progress diagnostics. |
| ModeScheduler | Enumerate keep/single-switch branches; enforce dwell and switch hysteresis before selection. |
| ModeManager | Commit a frozen entry request; gate handover on its matching ID and measured state. |
| ModeExecutor | Persist actual mode; supervise braking/alignment, acknowledge requests and latch execution faults. |
| Optimizer | Optimize continuous controls inside one branch; advance only an accepted keep-mode warm start. |
| NoiseGenerator | Seeded Gaussian proposals, effective projected perturbations and control-noise correction. |
| RolloutEngine | Generate an inspectable pose horizon and mark ticks consumed by transitions. |
| DriveModel | Mode constraints, braking/steering interlock and rate-limited wheel/body propagation. |
| Kinematics | Mechanical steering limits, signed wheel directions and encoder forward kinematics. |
| TransitionModel | Predict braking, mode-entry steering and confirmation delay. |
| CriticManager | Combine path distance/heading, circle-obstacle, goal, effort, smoothness and switch objectives. |

All configuration-bearing components store values instead of references into
other objects. Models and stateful controllers can therefore be constructed from
temporary configuration values and safely copied. Individual controller objects
are not thread-safe and must be serialized by their caller.

## Planning and execution

1. Check the timestamp, finite state, measured joint limits and path.
2. Update path progress and local target; reset task state for a new path/ID/policy.
3. If a committed mode transition is active, update it from actual measured feedback.
4. At the terminal pose or a nearby sharp corner, use bounded deterministic capture;
   otherwise finish local alignment or enumerate keep/single-switch branches.
5. Optimize each branch independently; never average controls across modes.
6. Reject switch candidates that fail hysteresis before selecting the winner.
7. Commit an immediate switch, or execute the first control in the current mode.
8. Advance a warm start only for an emitted keep-mode MPPI drive action.
9. Publish navigation status, progress, goal errors and the completion latch.

Terminal/corner capture is a separate low-speed policy in Controller, using the
same DriveModel, RolloutEngine collision checks and measured ModeManager protocol.
It avoids resampling a near-zero MPPI command at the completion boundary. GoalManager
only judges measured state; it never treats a predicted endpoint as completion.
See docs/NAVIGATION.md for the task and heading contracts.

Large same-mode steering changes commit a fixed control intent until alignment
and the first drive tick complete, with the same configured timeout bound. New
sampled directions cannot interrupt this operation. RolloutEngine predicts the
same commitment, masks controls ignored during it, and rejects a planned mode
switch that would preempt it. The initiating control remains active because it
sets entry geometry. Controller rechecks a constant-intent continuation against
fresh obstacles while local alignment is active; this check is conservative and
can stop before a shorter continuation would become unsafe. Reset clears the
commitment. Replanning clears obsolete local alignment, but never mutates an
active explicit mode request. A confirmed mode switch preserves its agreed entry
intent through the stopped handover; normal tracking then applies the first drive
intent. A new path or terminal/corner capture may choose a fresh intent afterward,
subject to the same stopped realignment interlock.

A rejected immediate switch uses the keep-mode solution, rather than projecting
the rejected mode's first control into the current mode. Future transitions
remain plans and require a fresh decision before execution. Future-switch plans
do not become cached keep-mode sequences. Faults and invalid inputs stop drive.

ModeManager observes both body velocity and every wheel speed before requesting
a mode. Matching actual_mode is insufficient without mode_confirmed, the current
request ID and measured entry steering. A confirmed switch produces a stopped handover
cycle before drive resumes. Retries cannot change the committed steering or renew
the deadline. Request IDs remain monotonic across deliberate recovery.
Timeout covers the entire committed transition, including braking.

## Continuous optimization

The proposal is the nominal sequence plus stationary AR(1) Gaussian perturbations,
projected into
the branch's mode and wheel-speed limits. One sample preserves the nominal
sequence; tracking also reserves one proposal for the fresh geometric seed,
so a shifted local target can replace stale warm-start direction. Ackermann seeds
use signed pure-pursuit curvature; Crab seeds use body-frame translation. The
approach speed cap applies to every proposal and the weighted sequence.
Updates use the effective perturbation after projection, not discarded
raw noise. Transition ticks are masked from the control-noise correction and
weighted update because no sampled drive control is applied during those ticks.
The control at switch_step defines a frozen entry intent. NoiseGenerator preserves
that control across all proposals, so masked noise cannot alter entry geometry.
Entry intent currently comes from the branch seed; optimizing discrete entry
directions as separate branches remains future work.

`noise_correlation` controls temporal continuity without changing marginal noise
variance. Zero selects independent noise. The noise process restarts after the
frozen mode-entry control. Weighting whitens nominal controls and effective noise
with the AR(1) precision operator inside each contiguous active segment; inactive
ticks break segments and contribute no correction. For zero correlation this
reduces to control_correction_weight * nominal * effective_noise / variance.
A disabled noise dimension contributes no correction. Weights use a
minimum-normalized exponential of trajectory cost plus that correction. Mode selection compares physical
critic scores; proposal correction is used only for weighting within a branch.
The output is the re-evaluated weighted sequence. A feasible nominal or sampled
sequence is used only when the weighted sequence is infeasible or no finite
weights exist. This remains a practical hybrid proposal, not a formal derivation
of an importance sampler over discrete mode changes.

Critic is a small ROS-independent C++ interface. CriticManager installs the
default objectives and allows additional const critics through shared ownership.
A nonfinite critic result rejects a trajectory. Each rollout exposes its initial
pose and one pose per tick, final vehicle state, applied controls and transition
mask. The obstacle critic checks swept centre-line segments with a circular
footprint, including braking and stationary alignment.

## Motion prediction

Inverse kinematics chooses only mechanically reachable joint angles and adjusts
wheel speed sign for equivalent directions. Distances are direct joint distances,
not wrapped shortcuts across hard stops. Every control is uniformly scaled when
wheel-speed limits would be exceeded, preserving its body twist direction.

Within a stable mode, joint changes within drive_steering_limit_rad use bounded
continuous steering. Steering and wheel speeds share an interpolation factor,
limited by wheel acceleration and steering rate. The model then checks the full
encoder-derived twist change, including the steering contribution, and reduces
the factor until linear/angular acceleration or deceleration limits hold.
The search has a bounded iteration count and rejects an invalid step.

Larger changes use brake/align/drive: steering stays fixed until body and wheel
feedback are stopped. The local commitment prevents stochastic target chasing.
Hard-stop crossings still require this stopped realignment. Mode transitions
retain their separate explicit request and measured confirmation protocol.

Forward kinematics of the next wheel speeds and next steering positions gives
the predicted twist, integrated using constant-twist SE(2) integration. Drive
outputs carry exactly those joint targets and that predicted body twist. The
executor verifies their consistency and moving-steering bounds. Measured wheel
speeds remain authoritative initial conditions; odometry is also checked by the
stopped gate. The adapter must supply mutually consistent feedback and convert
joint angular speeds to linear rolling speeds.

Transitions consume braking, bounded mode-entry alignment and confirmation ticks.
Even zero configured delays consume at least one tick to guarantee termination.
Entry steering comes from the branch's intended control using the same bounded
kinematics in prediction and execution. Crab aligns directly to its translation
direction; Ackermann can enter its planned curvature. Zero intent uses the
canonical mode geometry. Predicted transitions exceeding the execution deadline
are infeasible, even when they fit inside the planning horizon.

ModeExecutor is an optional standalone reference supervisor, not a dynamics
model. It consumes Output and measured VehicleState, returns joint targets and
ModeFeedback, and never substitutes predicted state for actual confirmation.
It blocks drive during transitions, retains steering while braking, persists
mode on zero drive, rejects changed/replayed requests and latches SafeStop.
The actuator layer owns rate limiting, encoder sampling and command watchdogs.
See docs/EXECUTION_CONTRACT.md for the API and reset protocol.

## Boundaries for simulation integration

The geometry and actuator defaults correspond to the simulation configuration.
The predictive drive model permits bounded continuous steering but retains
stopped realignment for larger changes. Braking response, body limits, slipping,
communication delay and transition times require identification before claiming
model agreement.

The simulator currently infers mode from velocity and clears it on zero commands;
this core requires an explicit persistent mode request and acknowledgement.
Integration must resolve that mismatch. Do not translate RequestMode to an
ordinary zero Twist or report confirmation based on elapsed time alone.

The next adapter should own message/TF conversion, feedback ages, simulation time,
task IDs, transport for the mode command/feedback protocol and deliberate recovery.
A later Nav2 adapter adds lifecycle/cancellation handling, path frame transforms
and costmap/footprint queries. Core PathManager now owns ordered path progress and
pruning; GoalManager owns pose/stop completion. The core still assumes one call
per fixed model tick and bounded localization displacement. SafeStop supports
execution cancellation, while action-server cancellation and recovery policy
remain adapter responsibilities.

## Validation

CMake/CTest cover mode restrictions, bounded inverse/forward kinematics, wheel
and body rate limits, braking before steering, measured wheel-stop confirmation,
zero-delay transition termination, all six directed mode transitions, request
identity/idempotency, timeout/recovery, persistent mode on zero drive, a typed
controller/executor lateral closed loop, hysteresis rejection, weighted-noise
projection, critic extension, swept collisions and deterministic straight-path
closed-loop progress. CTest also installs the library into a clean prefix and
builds/runs an independent consumer through find_package().

These tests validate core contracts, not Gazebo tracking performance. The next
stage must compare predicted trajectories against independent Gazebo truth and
measure solve-time distributions, tracking error, mode-switch counts, stalls and
faults.

Default-noise behavior regressions cover nine scenarios and five fixed seeds,
including reverse travel, terminal yaw, S-curves, reversals and a closed square.
An independent encoder fixture with midpoint pose integration checks completion,
measured stopping, one second of post-completion Hold, path error, mode changes
and unfinished stationary intervals. These are standalone regressions, not a
calibrated plant or Gazebo benchmark. See docs/VALIDATION.md for measured results.
