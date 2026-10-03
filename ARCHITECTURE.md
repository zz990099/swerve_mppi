# Core architecture

The library builds as swerve_mppi::core using CMake. Its public headers contain
no ROS, Nav2, Gazebo or pluginlib dependencies. ROS validation nodes and a future
Nav2 controller plugin belong in separate adapter packages.

## Components

| Component | Responsibility |
| --- | --- |
| Controller | Validate input, coordinate path/goal state, continue committed transitions and emit a body-velocity command or withhold authorization. |
| PathManager | Monotonic arc progress, bounded matching, local reference, effective target and terminal eligibility. |
| GoalManager | Pose acquisition, measured-stop settling, completion latch and progress diagnostics. |
| ModeScheduler | Enumerate keep/single-switch branches; enforce dwell and switch hysteresis before selection. |
| ModeManager | Commit a frozen entry request; gate handover on its matching ID and measured state. |
| ChassisExecutor | Compile body velocity and immutable mode-entry intent with DriveModel; synchronous reference supervision without transport/obstacle guards. |
| ModeExecutor | Persist actual mode; supervise braking/alignment, acknowledge requests and latch execution faults. |
| TimingGuard | Check clock/cadence, observation and command age, source/scheduled execution metadata and ordered envelopes. |
| TimedExecutor | Compile the body command at current measured state, transactionally preview execution, validate its exact joint interval/full stop against current context, commit or use checked braking; latch timing/protocol/unsafe-stop faults. |
| ActuationModel / ActuationPlan | Build a checked actuator reference and expose affine Drive or proportional braking followed by stationary alignment samples. |
| ProfileRunner | Serial high-rate sampling, joint-unit conversion and latched application/wall-clock expiry; leaves mode protocol to TimedExecutor. |
| FeedbackCheck / PlanningBudget | Admit mutually consistent body/joint observations and share one cooperative steady-clock budget across all branches. |
| Optimizer | Optimize continuous controls inside one branch; advance only an accepted keep-mode warm start. |
| NoiseGenerator | Seeded Gaussian proposals, effective projected perturbations and control-noise correction. |
| RolloutEngine | Generate an inspectable pose horizon and mark ticks consumed by transitions. |
| DriveModel | Mode constraints, braking/steering interlock and rate-limited wheel/body propagation. |
| Kinematics | Mechanical steering limits, signed wheel directions and encoder forward kinematics. |
| TransitionModel | Predict braking, mode-entry steering and confirmation delay. |
| CriticManager | Combine path distance/heading, circle-obstacle, goal, effort, smoothness and switch objectives. |
| TrajectoryValidator | Enforce finite trajectories, swept circular collision checks and injected hard constraints across motion policies. |

Controller hides its joint prediction in a private implementation; its public
Output contains optional ChassisCommand and PlanningDiagnostics only. A valid zero
velocity is a normal stop; absence cancels and latches execution. Explicit mode
requests freeze a body entry intent, not wheel angles. Execution compiles the
intent with the same model and retains the resulting joint geometry for retries.
Action and JointCommand are lower-layer representations, never planner output.
See docs/CHASSIS_COMMAND.md for migration and ownership.

All configuration-bearing components store values instead of references into
other objects. Models and stateful controllers can therefore be constructed from
temporary configuration values and safely copied. Controller and Optimizer own mutable reusable workspaces and are not thread-safe;
serialize calls to each instance. Returned Solution and Trajectory values own
independent copies and can safely outlive the next solve. Copies of controllers
and optimizers do not alias workspace storage. Reset clears task/RNG/warm-start
state while retaining vector capacity.

## Planning and execution

1. Start the shared wall budget; reject oversize contexts, then check timestamp, finite state, measured joint limits, body/joint consistency and path.
2. Update path progress and local target; reset task state for a new path/ID/policy.
3. If a committed mode transition is active, update it from actual measured feedback.
4. Finish committed local alignment/first Drive before choosing a new terminal,
   corner or tracking policy; a replan may cancel obsolete local intent.
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

PathReference carries target, target_kind, target_remaining_m and goal_eligible.
An uncaptured corner owns translation and its speed cap; a nearby global endpoint
cannot take over. Controller and GoalManager both consume the same eligibility.
Lookahead targets retain normal tracking limits until a corner or eligible goal
requires slowdown. The global goal remains the navigation diagnostic/completion goal.

Large same-mode steering changes commit a fixed control intent until alignment
and the first drive tick complete, with the same configured timeout bound. New
sampled directions cannot interrupt this operation. RolloutEngine predicts the
same commitment, masks controls ignored during it, and rejects a planned mode
switch that would preempt it. The initiating control remains active because it
sets entry geometry. After an explicit switch, its first Drive remains masked
because it executes the frozen entry control, not a proposal at the resume index.
Controller rechecks alignment, first Drive and a full stopping continuation against
fresh constraints while local alignment is active. Reset clears the
commitment. Replanning clears obsolete local alignment, but never mutates an
active explicit mode request. A confirmed mode switch preserves its agreed entry
intent through the stopped handover; normal tracking then applies the first drive
intent. Terminal/corner policies also pass through the shared control application
and committed alignment path, with the same deadline and obstacle recheck. They
may choose a fresh intent after the first Drive. A new task path/reset or a checked
planning stop clears obsolete local intent; explicit mode requests remain immutable.

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
with the marginal AR(1) precision operator over active ticks. Inactive ticks
contribute no correction but retain correlation rho^gap between successive active
indices; only the frozen explicit mode-entry tick breaks this linkage. Callers
of NoiseGenerator::correction must pass the same Branch used for sampling.
For zero correlation this reduces to control_correction_weight * nominal * effective_noise / variance.
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
mask. Controller and Optimizer share one const TrajectoryValidator. Its swept
centre-line circle checks require an initial pose matching current feedback within
1e-9 m/rad (yaw modulo 2*pi), independently check the exact current footprint,
and reject stale or shifted traces. These checks and injected TrajectoryConstraint
objects apply to MPPI,
capture, alignment and fallback stopping. Build/configure the validator before
passing it to Controller; do not mutate shared constraints during computation.
The obstacle critic contributes clearance cost; lowering a weight cannot override
a hard rejection.

When planning has no feasible result, Controller discards warm/local intent and
checks zero-control braking using stopping_horizon_steps (default 200), independently
of the MPPI horizon. Only a valid trace ending with exactly zero modeled body and
wheels can produce recoverable Brake/Hold. Otherwise
UnsafeStoppingTrajectory produces SafeStop. Waiting/Blocked describes a checked
planning stop awaiting fresh input, not completed navigation.

Every healthy Brake/Hold/RequestMode passes a final non-driving safety check,
including normal capture early returns and committed mode handshakes. In 0.14.1,
ActuationModel::plan_stopping models the actual first interval: proportional braking
with retained steering, then any permitted Hold/RequestMode alignment in remaining
stopped time. RolloutEngine::generate_execution appends its complete stopping tail.
A brake-only trace must not authorize steering against fresh hard constraints.
A valid normal stop retains its navigation status and immutable request payload;
rejection clears the payload and emits UnsafeStoppingTrajectory/SafeStop.
The nominal plan preserves measured mode, request ID and confirmation; it does not
preview or commit the executor protocol. TimedExecutor remains the final authority
for actual execution-time state and protocol effects. Ordinary driving rollouts
still require confirmed feedback. Stopping checks do not optimize motion.

## Workspaces and diagnostics

Optimizer preallocates proposal noise, active masks, candidate/weighted controls
and reuses one rollout trace. It copies trajectory data only for nominal/weighted
results or an improved safety fallback, preserving value ownership. RolloutEngine
also exposes a caller-owned output overload; controls must not alias its output
controls. A new rollout clears validity and vector contents while retaining
capacity. Transition traces append directly to the preallocated pose buffer.

Controller retains RolloutEngine, a shared hard validator and control storage for
capture/alignment safety checks. It copies only the pruned path into planning
input. PathManager starts geometry scans with arc-length binary lookup and stops
at the match/lookahead bounds; full input validation and geometry identity checks
still scan the supplied task path. PathCritic compares squared distances and
interpolates body yaw only at the nearest segment.

Output::control_policy identifies Tracking, Alignment, Capture, ModeTransition,
Stopped, Blocked or Fault. FailureReason separates invalid data, clock/path/feedback
faults, no feasible plan, unsafe stopping, model failure and transition faults. PlanningStats counts all
branches, physical rollout evaluations, finite scores and fallback updates in the
current compute call, with budget_exhausted indicating cooperative deadline rejection.
Controller uses one steady-clock budget across preparation, all branches and final
safety/publication checks. Optimizer checks between evaluations; a single bounded
rollout or custom critic can overrun. A final gate rejects every expired result,
including terminal Hold. The independently scheduled actuator watchdog must stop
execution while a solve is delayed; arbitrary user callbacks must themselves be bounded.
Legacy Output::feasible_rollouts remains the selected branch's feasible proposal
count. See docs/PERFORMANCE.md for measurement and exact counter semantics.

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

Forward kinematics of the next wheel speeds and steering positions gives the
endpoint twist. A Drive interpolates both joint arrays affinely over the entire
tick, including deceleration and signed reversals. Fixed steering gives an affine
body twist; commuting twists integrate exactly in SE(2). Moving steering uses
analytic encoder-vector integrals for yaw and eight midpoint intervals for
translation, with a world-acceleration bound enclosing quadrature error and
curve-to-chord deviation. Brake retains measured steering and proportionally
reduces the measured wheel speeds at the fastest common allowed body/joint rate,
then holds zero for the remainder of the tick. These are actuator contracts that
an adapter must implement and calibrate.
The executor checks endpoint absolute mode speed limits separately from manifold
projection tolerance, plus pointwise body and joint rates during Drive interpolation.
Measured overspeed enters the separately validated Brake path. Measured encoders
remain authoritative initial conditions; odometry is also checked by the stopped
gate. The adapter must supply mutually consistent feedback and convert joint
angular speeds to linear rolling speeds.

Transitions consume braking, bounded mode-entry alignment and confirmation ticks.
Even zero configured delays reserve two post-alignment cycles: executor
confirmation Hold, then manager measured-feedback handover Hold. The receipt
deadline precedes the end of handover by one tick.
Entry steering comes from the branch's intended control using the same bounded
kinematics in prediction and execution. Crab aligns directly to its translation
direction; Ackermann can enter its planned curvature. Zero intent uses the
canonical mode geometry. Predicted transitions exceeding the execution deadline
are infeasible, even when they fit inside the planning horizon.

ChassisExecutor is the synchronous body-command reference entry point. It compiles
Output with DriveModel at measured VehicleState, then uses the low-level ModeExecutor
to supervise the resulting JointCommand. The latter returns joint targets and
ModeFeedback, and never substitutes predicted state for actual confirmation.
It blocks drive during transitions, retains steering while braking, persists
mode on zero drive, rejects changed/replayed requests and latches SafeStop.
The actuator layer owns rate limiting, encoder sampling and command watchdogs.
See docs/EXECUTION_CONTRACT.md for the API and reset protocol.

Every Drive policy uses the private planner's apply_control first-Drive plus complete-stop
gate before output and warm-start acceptance. RolloutEngine::generate_continuation
retains committed entry/alignment intent until that first Drive, then brakes fully
to zero. If the preferred continuation is rejected, up to
safety_reduction_attempts amplitude halvings per candidate revalidate the entire
continuation, retaining direction/curvature, alignment clock and entry geometry.
A successful reduction clears optimizer warm state; Output::safety_reductions
counts all reduced validations in the compute call. Exhausting the retries uses
the separately validated current stop. Exhausting stopping_horizon_steps fails closed. Stop thresholds are used
for measured execution handover, not to discard residual predicted displacement.
Zero intent is explicitly Brake/Hold and retains measured steering and zero drive
targets; measured module residuals do not incorrectly turn braking into Drive.
Shared inclusive time comparisons use a 1 ns floor or four times double precision epsilon times
the timestamp magnitude, whichever is larger. Duration-to-ticks rounding uses the
same tolerance. Timestamp order and replay protection remain strictly increasing.

Version 0.12 requires fresh confirmed/matching mode feedback for Stable Drive.
Whole-period absolute speed certification shares analytic encoder derivatives
between prediction and execution, with bounded adaptive enclosures and fail-closed
exhaustion. When needed, the model reduces rolling targets to preserve steering
progress at a speed cap. Hold/RequestMode complete retained-angle proportional
braking before steering in the remaining tick time. Transition prediction retains
this residual displacement during alignment and confirmation. Local Ackermann
curvature walks nonzero geometric segments, preserving caps across duplicate points.

## Boundaries for simulation integration

The geometry and actuator defaults are standalone configuration assumptions.
The predictive drive model permits bounded continuous steering but retains
stopped realignment for larger changes. Braking response, body limits, slipping,
communication delay and transition times require identification before claiming
model agreement.

An adapter must preserve the explicit persistent mode request and acknowledgement
contract. Do not infer mode from velocity, clear mode on zero commands, translate
RequestMode to an ordinary zero Twist or report confirmation from elapsed time alone.

TimingGuard and TimedExecutor provide reusable checks for cadence, feedback ages,
command expiry/replay and session renewal without transport/ROS dependencies.
They cannot detect process silence without calls; actuators need an independent
watchdog. Direct ChassisExecutor is the synchronous body-command reference entry point,
without command envelopes or independent trajectory admission. See docs/EXECUTION_CONTRACT.md for the guarded API.

The next adapter should own message/TF conversion, feedback timestamps, simulation time,
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

Default-noise behavior regressions cover fourteen scenarios and five fixed seeds,
including reverse travel, terminal yaw, S-curves, reversals and a closed square.
An independent encoder fixture with 64 substeps per tick checks completion,
measured stopping, one second of post-completion Hold, path error, mode changes
and unfinished stationary intervals. These are standalone regressions, not a
calibrated plant or Gazebo benchmark. See docs/VALIDATION.md for measured results.


## Execution-time validation (0.14)

Controller validation certifies a plan at its planning observation. A queued plan
must be revalidated at execution: the robot or obstacles may have changed during
computation/transport even when all timestamps remain within their age limits.
TimedExecutor takes current ControllerInput and a source/scheduled/expiry-stamped
CommandEnvelope. TimingGuard remains separately usable for transport checks.
The envelope owns its originating task ID, heading policy and full ordered path.
TimedExecutor rejects a missing/mismatched snapshot before any supervisor preview,
even if the old command remains mechanically and collision valid. TaskMismatch uses
the same independently checked stopping fallback; a fresh command for the latest
task can recover without resetting healthy execution.

The supervisor is copied for a transactional preview. ActuationModel converts that
ExecutionResult and the execution-start state into a checked ActuationPlan.
RolloutEngine::generate_execution integrates this exact joint interval and a full
stop; the shared TrajectoryValidator applies current circular obstacles and
injected hard constraints. Only a valid continuation commits the preview. A rejected
command gets a separate checked stopping preview; unsafe stopping latches SafeStop.
New request IDs are not consumed by a rejected preview. Healthy output contains the
same ActuationPlan the validator checked, for high-rate target sampling.

The actuator reference is production code. The behavior fixture retains independent
encoder equations/integration as a test oracle. No ROS/Gazebo transport, global
reacquisition or physical actuator calibration is added here. Version 0.15 adds
cooperative deadlines and a portable profile consumer, still without ROS transport.
The current state must describe the application instant; estimated states and
uncertainty envelopes require an explicit adapter/model contract. A timestamp
rewrite alone does not time-align stale measurements.
