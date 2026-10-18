# Architecture

## Control boundary

`Controller::compute(Input)` returns a desired body velocity and explicit target
mode. It has no task-completion output, command ID, application-time receipt,
publication expiry, recovery service or independent trajectory-validation pipeline.
`reset()` clears warm-start and observed-mode age when the caller changes tasks.
Invalid input or an unusable candidate batch returns a zero command plus status;
the next valid call can compute again without a recovery handshake.

The core public headers are `types.hpp`, `config.hpp`, `model.hpp`, `critic.hpp`
and `controller.hpp`. It consumes no ROS types and builds independently with CMake.
The ROS package links the installed `swerve_mppi::core` target.

## Optimization

Each compute starts from current measured pose, velocity, steering and wheel speeds.
Candidate branches retain the current mode or request one immediate switch to either
other mode, after minimum mode dwell. Each branch samples temporally correlated
velocity sequences, projects them onto its mode's velocity/curvature constraints,
rolls out nominal response and accumulates batch critic costs. Exponential cost
weights produce an updated sequence. The weighted result is rescored; a better
sample remains the fallback. Branch selection includes switching cost and hysteresis.
This is a constrained sampling MPPI implementation, not a claim of globally optimal
mode scheduling or closed-loop stability guarantees.

The same-mode warm start advances by elapsed model intervals. New measured modes,
task resets and switching clear it. There is no reconstruction of private chassis
command history. Near the final target, noise and switching penalties taper with
pose error to avoid a fixed switching dead zone. A steering-change critic discourages
mechanical +/-pi/2 discontinuity crossings within the same mode.

`State::switching` and `requested_mode` describe the currently executing transition.
While switching, core returns zero velocity in that requested mode and does not pick
a conflicting branch. The ROS adapter exposes its pending request as switching even
before the chassis echoes it. The first call assumes an already-established initial
mode; later observed mode changes restart minimum dwell.

## Nominal model

Body velocity is constrained by mode: Ackermann has `vy=0` and bounded curvature;
Spin has only `wz`; Crab has only `vx,vy`. Inverse kinematics chooses a signed wheel
representation within steering hard stops. Response integrates measured/nominal
velocity, applies velocity and wheel/steering slew, and models brake-before-align
with alignment dwell for mode changes and large same-mode steering changes.

The model is reanchored to measurements each compute. It does not reproduce the
Python executor's hidden state or watchdogs, and does not model tire contact,
actuator torque or general slip. Offline tracking against the real Python chassis
checks this approximation separately from numerical model unit tests.

## Critics and maps

`Critic::prepare` runs once per control call. `score` adds costs to an entire batch;
infinity rejects a candidate. Core supplies numerical path-distance, path-heading,
progress, terminal position/yaw, effort, smoothness and steering-change costs in a
default tracking critic. An application can replace or extend the critic list.

`GridView` borrows an axis-aligned immutable 0..254 cost grid with 255 as unknown.
The caller owns storage for the complete compute call. The ROS-side CostmapCritic
demonstrates injection without ROS dependencies in core. It scores centers in an
already inflated map; it is not a swept-footprint collision validator. Physical
obstacle navigation is outside first-stage acceptance.

## Nav2 caller responsibilities

The ROS integration is a plugin loaded by Nav2's controller server, not an
independent node with a control loop. Nav2 supplies robot pose/body velocity and
owns FollowPath actions, frequency, cancellation and goal evaluation. The plugin
receives chassis joint/mode feedback and uses source timestamps plus wall silence
for freshness. The local costmap is copied under its mutex for each compute call.

Paths remain in their source frame for monotonic progress; each prepared local
section and endpoint is transformed into the current costmap global frame. Sections
exclude later return legs. Endpoint advancement requires nearby path geometry as
well as forward projection. `Input::goal_active` permits terminal attraction only
on the last ordered segment.

`Nav2Controller` returns the velocity that Nav2 publishes through its normal
cmd_vel chain. A separate command-context publisher carries explicit mode/request
ID/entry geometry. `chassis_bridge` combines that context with the final Nav2
velocity; context alone cannot drive, and stopping/cancellation cannot be bypassed
by publishing core speeds directly to the chassis.

The plugin retains pending mode requests until measured confirmation, including
before the chassis echoes the request. Retried entries remain frozen. Faulted
startup uses the same zero-request path. No ordinary speed receipt is required.

`OrderedGoalChecker` is separately loaded through Nav2's GoalChecker interface.
Its normal position/yaw/speed settling also requires final ordered progress and
mode readiness supplied by the controller. It starts closed after reset because
Nav2 checks the goal before computing its first command. This prevents closed
paths from completing at startup. The standalone numerical GoalChecker helper is
retained for portable regression tests, not as a competing ROS task engine.

Core remains 0.28.0; ROS 0.29.0 deliberately replaces the old standalone ROS API.
See docs/ROS.md for Jazzy-specific plugin configuration and bridge routing.
