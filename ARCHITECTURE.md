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

## Caller responsibilities

The ROS node receives the latest state/odometry, transforms a new path into the
odometry frame, preserves ordered progress, prepares a local path and calls core
at a configured frequency. Original encoder stamps are used for freshness checks;
same-stamp mode/fault updates are accepted. There is no atomic snapshot pairing gate.
Normal speed commands are published continuously without a receipt service.

The small chassis adapter preserves the accepted request ID and entry geometry.
A mode change becomes a zero-speed request with frozen entry velocity. Retries keep
the same request until actual mode/ready feedback confirms completion. This feedback
is mode state, not an acknowledgement of every speed command. A faulted startup
chassis receives a new zero request through the same adapter; there is no separate
recovery API. Other processes must not concurrently own the command topic.

GoalChecker is outside core and requires endpoint position, yaw and measured speed
within caller-configured bounds for a settle interval. It then ends the task and
publishes ordinary zero commands. Terminal critics guide convergence but do not
declare success. New tasks reset both path progress and goal-checking state.
