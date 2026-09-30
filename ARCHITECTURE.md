# Core architecture

The library builds as swerve_mppi::core using CMake. Its public headers contain
no ROS, Nav2, Gazebo or pluginlib dependencies. ROS validation nodes and a future
Nav2 controller plugin belong in separate adapter packages.

## Components

| Component | Responsibility |
| --- | --- |
| Controller | Validate input, continue committed transitions, select a solution and emit one action. |
| ModeScheduler | Enumerate keep/single-switch branches; enforce dwell and switch hysteresis before selection. |
| ModeManager | Execute the request/confirmation handshake, measured-stop gate and latched timeout. |
| Optimizer | Optimize continuous controls inside one branch; advance only an accepted keep-mode warm start. |
| NoiseGenerator | Seeded Gaussian proposals, effective projected perturbations and control-noise correction. |
| RolloutEngine | Generate an inspectable pose horizon and mark ticks consumed by transitions. |
| DriveModel | Mode constraints, braking/steering interlock and rate-limited wheel/body propagation. |
| Kinematics | Mechanical steering limits, signed wheel directions and encoder forward kinematics. |
| TransitionModel | Predict braking, mode-entry steering and confirmation delay. |
| CriticManager | Combine path, circle-obstacle, goal, effort and switch objectives. |

All configuration-bearing components store values instead of references into
other objects. Models and stateful controllers can therefore be constructed from
temporary configuration values and safely copied. Individual controller objects
are not thread-safe and must be serialized by their caller.

## Planning and execution

1. Check the timestamp, finite state, measured joint limits and path.
2. If a committed transition is active, update it from actual measured feedback.
3. Enumerate candidate discrete branches. Each makes at most one transition.
4. Optimize each branch independently; never average controls across modes.
5. Reject switch candidates that fail hysteresis before selecting the winner.
6. Commit an immediate switch, or execute the first control in the current mode.
7. Advance a warm start only for an emitted keep-mode drive action.

A rejected immediate switch uses the keep-mode solution, rather than projecting
the rejected mode's first control into the current mode. Future transitions
remain plans and require a fresh decision before execution. Future-switch plans
do not become cached keep-mode sequences. Faults and invalid inputs stop drive.

ModeManager observes both body velocity and every wheel speed before requesting
a mode. Matching actual_mode is insufficient without mode_confirmed. A
confirmed switch produces a stopped handover cycle before drive resumes.
Timeout covers the entire committed transition, including braking.

## Continuous optimization

The proposal is the nominal sequence plus Gaussian perturbations, projected into
the branch's mode and wheel-speed limits. One sample preserves the nominal
sequence. Updates use the effective perturbation after projection, not discarded
raw noise. Transition ticks are masked from the control-noise correction and
weighted update because no sampled drive control is applied during those ticks.

Weights use a minimum-normalized exponential of trajectory cost plus
control_correction_weight * nominal * effective_noise / variance. A disabled
noise dimension contributes no correction. Mode selection compares physical
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

Within a stable mode, a significant steering change uses a conservative
brake/align/drive sequence. Steering remains fixed until body and wheel feedback
are stopped. Wheel acceleration, body linear acceleration/deceleration and angular
acceleration/deceleration bound a common wheel-speed interpolation factor.
Forward kinematics of those wheel speeds and measured steering angles gives the
predicted body twist, which is integrated using constant-twist SE(2) integration.
Measured wheel speeds are authoritative for propagation; odometry velocity is
also checked by the stopped gate. The adapter must supply mutually consistent
feedback and converts joint angular speeds to linear rolling speeds.

Transitions consume braking, bounded mode-entry alignment and confirmation ticks.
Even zero configured delays consume at least one tick to guarantee termination.
Crab entry currently aligns to zero steering, then a drive intent can cause a
second alignment to its translation direction. A future typed executor contract
should carry the desired entry direction to avoid that extra phase.

## Boundaries for simulation integration

The geometry and actuator defaults correspond to the simulation configuration.
The predictive drive interlock remains more conservative than the simulator's
active double-Ackermann steering behavior. Braking response, body limits, slipping,
communication delay and transition times require identification before claiming
model agreement.

The simulator currently infers mode from velocity and clears it on zero commands;
this core requires an explicit persistent mode request and acknowledgement.
Integration must resolve that mismatch. Do not translate RequestMode to an
ordinary zero Twist or report confirmation based on elapsed time alone.

The next adapter should own message/TF conversion, feedback ages, simulation time,
path lifecycle, explicit mode command/feedback and deliberate recovery. A later
Nav2 adapter adds lifecycle handling, local path transformation/pruning,
costmap/footprint queries and goal completion. The core has no autonomous handling
of arbitrary control periods, global path progress or navigation cancellation yet.

## Validation

CMake/CTest cover mode restrictions, bounded inverse/forward kinematics, wheel
and body rate limits, braking before steering, measured wheel-stop confirmation,
zero-delay transition termination, hysteresis rejection, weighted-noise
projection, critic extension, swept collisions and deterministic straight-path
closed-loop progress. CTest also installs the library into a clean prefix and
builds/runs an independent consumer through find_package().

These tests validate core contracts, not Gazebo tracking performance. The next
stage must compare predicted trajectories against independent Gazebo truth and
measure solve-time distributions, tracking error, mode-switch counts, stalls and
faults.
