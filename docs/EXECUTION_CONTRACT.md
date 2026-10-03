# Standalone execution protocol (0.18)

This contract is independent of ROS, Nav2 and Gazebo. TimedExecutor is the guarded
integration entry point; execute only its returned ActuationPlan, sampled at the
actuator rate. ModeExecutor is a low-level protocol supervisor used by that entry
point and by core tests. ChassisExecutor is the synchronous body-command reference
entry point; it omits the timed/context trajectory guards. Its endpoint arrays alone are not a complete checked
actuator reference. Neither class simulates actuators or certifies physical tracking.

## Request and feedback

| Field | Meaning |
| --- | --- |
| Output::command | Optional ChassisCommand; absent means cancellation/fault and independent stop. |
| ChassisCommand::target_velocity | Nominal body-frame vx/vy (m/s), wz (rad/s); valid zero is ordinary braking/holding. |
| ChassisCommand::mode | Explicit selected mode; ordinary velocity requires the measured actual mode. |
| ChassisCommand::mode_request | Optional complete immutable request; target_velocity must be exactly zero. |
| ModeRequest::id | Nonzero, monotonically increasing per serialized executor session. |
| ModeRequest::mode | Explicit desired mode; never inferred from zero/nonzero twist. |
| ModeRequest::entry_velocity | Frozen body intent defining entry geometry; never authorizes drive. |
| ModeFeedback::actual_mode | Last successfully completed mode, retained throughout a switch. |
| ModeFeedback::request_id | Executor's active/last accepted ID; completion also requires confirmed. |
| ModeFeedback::confirmed | Stable mode, with transition stop/alignment checks completed. |
| ModeFeedback::fault | Latched execution failure; requires deliberate recovery. |
| ModeFeedback::time_in_mode_s | Age of the actual mode; resets only when a new mode completes. |

VehicleState retains its public mode fields. Populate them from feedback as
shown below, alongside fresh measured pose, body velocity, steering and wheel
speeds. Request ID zero denotes startup state. ModeManager selects the next ID
above both its own high-water mark and VehicleState::mode_request_id. Both
manager and executor retain high-water marks across reset. A restarted process
must establish a fresh, drained transport session and obtain executor feedback
before enabling control; this is not a cross-process UUID protocol.

See [CHASSIS_COMMAND.md](CHASSIS_COMMAND.md) for command construction and migration.
TimedExecutor compiles a target with the shared DriveModel at the execution snapshot,
then validates that actual compiled interval and its complete stop. It does not
interpret the planner target as endpoint FK or allow a Twist-only mode switch.

## Cycle

```cpp
#include <swerve_mppi/planning/controller.hpp>
#include <swerve_mppi/execution/timing.hpp>
#include <swerve_mppi/execution/profile_runner.hpp>

swerve_mppi::Config config;
auto validator = std::make_shared<swerve_mppi::TrajectoryValidator>(config);
// Add bounded const hard constraints before sharing the validator.
swerve_mppi::Controller controller(config, validator);
swerve_mppi::TimedExecutor executor(config, session_id, initial_actual_mode, {}, validator);
swerve_mppi::ProfileRunner profiles(config, .5); // Monotonic wall-clock watchdog.
std::uint64_t sequence = 0;

// Once per config.dt_s. Capture the task from the actual planning observation.
const auto command = controller.compute(planning_input);
const swerve_mppi::CommandEnvelope envelope{
    session_id, ++sequence, issued_at_s, command,
    planning_input.vehicle.stamp_s, application_time_s, valid_until_s,
    swerve_mppi::CommandTask::capture(planning_input)};
// latest_input describes the application instant, including current constraints.
const auto guarded = executor.update(envelope, latest_input, application_time_s);
// A separately checked stopping fallback may also install. Failed/expired
// installation latches until verified recovery; it cannot retain old Drive.
if (!profiles.install(guarded, application_time_s, monotonic_wall_s)) {
  adapter.emergency_stop(); // Independent watchdog behavior, no certified profile.
}
// Map protocol feedback alongside the NEXT measured pose/twist/encoders:
const auto &feedback = guarded.execution.feedback;
next_input.vehicle.actual_mode = feedback.actual_mode;
next_input.vehicle.mode_confirmed = feedback.confirmed;
next_input.vehicle.mode_fault = feedback.fault;
next_input.vehicle.mode_request_id = feedback.request_id;
next_input.vehicle.time_in_mode_s = feedback.time_in_mode_s;
// Advance stamp_s from the actual measurement clock, not wall-clock prediction.
```

This is a mapping sketch: the input snapshots, truthful scheduling times, nonzero
session_id, verified initial_actual_mode and adapter functions belong to the caller.
The declarations are one-time setup; only the cycle repeats. All times use one
clock and source_stamp <= issued_at <= application_time <= valid_until.
latest_input.vehicle.stamp_s must describe application_time_s, not an old sample
whose timestamp was rewritten. For stepped tests, observe at the application
boundary; real-time integration requires explicit state time alignment.

The independently scheduled actuator callback samples the installed profile:

```cpp
const auto targets = profiles.sample(now_s, monotonic_wall_s);
if (targets) {
  // FL, FR, RL, RR: steering_angles in rad, wheel_angular_speeds in rad/s.
  adapter.publish_joint_targets(*targets);
} else {
  adapter.emergency_stop(); // Never extrapolate or hold an expired Drive.
}
```

Run serial executor ticks and a missing-command watchdog update even if planning
does not produce a result. Replace the profile at each validated application
boundary; no previous profile authorizes the next tick. Treat diagnostics from
guarded as authoritative: a rejected planner Output must not report completion.
Serialize install/sample/reset on each ProfileRunner (or protect with a short
lock); do not share its mutable state concurrently. Planning must not block the
actuator callback. Both times supplied to ProfileRunner must be nonnegative,
finite and nondecreasing; application time is simulation time and wall time is
steady time. A paused simulation still expires the wall watchdog. Missed tick
boundaries, malformed results and either clock rollback latch failure. Reset the
runner only after independently stopping/verifying the plant, draining old commands,
renewing the TimedExecutor session and resetting Controller. None of these helpers
detect total process silence without callbacks: the actuator endpoint needs an
independent watchdog/emergency stop. No normal braking profile certifies SafeStop.

Before planning or certifying a stop, `check_model_feedback` compares measured
body-frame twist with encoder forward kinematics and requires agreement within
1e-9 numerical precision. `check_feedback` retains configurable diagnostic defaults
of 0.05 m/s linear vector error and 0.10 rad/s angular error, but diagnostic admission
alone does not authorize nominal modelling. Even a diagnostic-valid discrepancy
returns InconsistentFeedback and cancels execution without a checked normal-stop
fallback. The model has no slip, localization or delayed-observation uncertainty
bounds. Never overwrite body measurements to pass admission. Encoder-derived
odometry agrees by construction and cannot independently validate physical slip.

A checked CommandRejected/TaskMismatch fallback may be Brake, Hold or RequestMode.
When a transition is already pending, Brake input continues its frozen stopped
alignment; ProfileRunner consumes that checked non-driving profile. Rejection does
not cancel the original request, change its ID/entry geometry or renew its deadline.
Faulted, unchecked, expired and rejected Drive profiles remain inadmissible.

Initialize the executor with verified actual mode; this does not measure or
automatically discover the chassis configuration.
For a nondefault startup request ID, initialize it through reset(recovered) before
normal updates. Do not run two supervisors against the same actuators. A hardware
or simulator supervisor can implement the same protocol instead of ModeExecutor,
but must also implement execution-time trajectory checks, timing/task guards and
the sampled actuator contract. Raw ModeExecutor is suitable for protocol tests;
it does not independently check obstacles or injected TrajectoryConstraint objects.

The first request is accepted only in Stable. A higher ID cannot replace an
active transition. The same ID and identical payload is an idempotent retry;
a changed payload, lower ID or malformed request latches a fault. Entry targets
must also match the requested mode: parallel wheel axes for Crab, tangential
axes for Spin, or a common Ackermann curvature respecting min_turn_radius_m.
A retry after completion returns a stopped acknowledgement without restarting mode age.
Replaying a completed request after the chassis has started moving is a fault.
In 0.4 a Drive's body_command must be finite and match forward kinematics of
wheel_speed_targets and steering_targets. These are the predicted next joint
positions/speeds, not a twist computed at the previous measured angles. Each
steering step is bounded by both drive_steering_limit_rad and
max_steer_rate_radps * dt_s relative to measurement. The actuator must track both
joint target arrays, including steering while driving; it must not gate all
steering changes on zero speed.

The mode-projection consistency allowance uses maximum commanded wheel speed
times drive_steering_limit_rad for linear error, and that bound divided by module
radius for angular error. This permits a bounded transient between steering
geometries; it is not a calibrated tire-slip model. Large steering jumps and
incompatible-mode body motion remain invalid. The model limits the complete
wheel/steering step against body acceleration, and the actuator layer continues
to own physical rate enforcement.

The transport must preserve command order and have one serialized command writer.

During braking, all drive targets are zero and steering retains measured angles.
Both body velocity and every measured wheel speed must be within their stopped
thresholds before the executor supplies the frozen entry steering. These thresholds
permit protocol handover, but do not erase residual rolling motion. In 0.12 the
actuator must finish the complete proportional brake at retained measured angles,
then align using only the remaining stopped time within that tick. Brake always
retains measured angles. Hold/RequestMode never steer while residual wheel speeds
are nonzero. The model accounts for this residual braking in alignment and
confirmation ticks, even when the initial geometry is already aligned. Confirmation requires measured
angles within steering_tolerance_rad and at least alignment_min_s since entering
alignment. If wheels/body move again, it returns to braking and restarts the
alignment minimum, while retaining the original overall deadline. The reported
mode changes only at confirmation. Controller/ModeManager then verifies mode,
ID, stopped measurements and equivalent rolling-line geometry modulo pi, and
emits one stopped handover cycle retaining measured steering. The executor's
own confirmation still requires its exact frozen mechanical targets.

## Internal actions, timing and cancellation

The following actions describe JointCommand/ExecutionResult in the lower execution
layer. Controller output has no action; the compiler derives these states from
target velocity, the explicit request, measured feedback and command presence.

| Internal action | Stable execution | During a committed transition |
| --- | --- | --- |
| Drive | Require fresh confirmed measured feedback matching the executor actual mode, finite twist/wheel targets, bounded moving-steering steps and consistency of the complete joint target step within mode limits. | Mask drive and continue the committed stop/alignment. |
| Brake | Zero drive targets, retain measured steering. | Continue the committed transition. |
| Hold | Zero drive; apply steering only when stopped, otherwise brake. | Continue the committed transition. |
| RequestMode | Start/retry the explicit request. | Retry the same request; reject replacement. |
| SafeStop | Zero drive and latch fault. | Cancel alignment, retain measured steering and latch fault. |

For every Drive, the executor checks each module's rolling vector against
(vx - wz*y_i, vy + wz*x_i), in addition to encoder/body agreement and mode limits.
The largest residual must not exceed drive_kinematic_tolerance_mps (default
0.02 m/s, nonnegative; zero permits only the 1e-9 m/s numerical allowance).
Since 0.19.1 this covers the entire affine Drive interval, including measured
start and every interior instant, rather than its target endpoint alone.
Prediction, ModeExecutor and ActuationModel share the bounded analytic certifier;
an observed excess or certification exhaustion rejects Drive. Moving steering
may be reduced or rejected with a zero/very small residual envelope. Fixed-angle
ideal motion and exact common translating wheel vectors remain admissible.
Aggregate least-squares agreement alone cannot validate mutually opposing wheels.
DriveModel applies the same residual envelope to nonzero, ready Drive steps while
limiting wheel/steering changes jointly. Braking keeps measured steering and does
not require a measured slipping wheel pair to become an ideal kinematic pair.
The allowance bounds the complete reference interpolation error; it is not a
tire-slip model or proof that the chassis will follow those targets.

An absent Output::command is the public cancellation/fault signal and compiles
to internal SafeStop. An empty or invalid path also withholds command authorization. Hold or a different DriveMode request cannot
cancel a committed transition or recover a fault.
Malformed measurements, nonincreasing timestamps and execution faults also latch
SafeStop. Executor fault outputs retain the last valid measured steering and
zero all drive targets; an actuator watchdog must enforce stopping even when the
calling process stops running. The core cannot detect silence without calls.

ModeManager's deadline starts when a switch is committed, including its braking.
ModeExecutor's deadline starts when it receives the first RequestMode, including
any local braking. Retries renew neither deadline. TransitionModel accounts for
braking and bounded alignment, then reserves max(2, ceil(confirmation_prediction_s
/ dt_s)) stopped cycles. These include the executor confirmation Hold and the
manager's measured-feedback handover Hold. The parameter is the total
post-alignment planning allowance: values below two ticks cannot remove protocol
cycles; larger values reserve additional feedback/transport latency. The default
0.20 s at dt_s=0.1 retains two cycles. It never permits synthesized confirmation.
The receipt deadline includes braking/alignment and confirmation receipt, before
the final handover cycle ends; first Drive may occur one tick after that deadline.
A horizon must still contain both stopped cycles. Predicted late receipt is
infeasible.
Feedback age, clock-reset handling and transport latency remain caller duties.
The reusable guarded entry point below enforces these checks when the caller
provides a current clock and periodic watchdog ticks.

Recovery is explicit: stop actuators independently, verify the actual mode and
stopped body/joints, drain old transport commands, call executor.reset(recovered)
with confirmed=true/fault=false, then controller.reset(). A moving, malformed or
unconfirmed recovery state is rejected. Reset does not reuse request IDs. No
fault is cleared merely because a late acknowledgement or target angle appears.

## Recoverable planning stops

NoFeasiblePlan is not itself an execution fault. Controller checks a complete
zero-control stopping trace through its hard TrajectoryValidator, including the
current footprint and every swept segment. If valid and stopped at its end, it
emits a valid zero body target with Waiting/Blocked, retains measured steering and permits fresh
replanning. Warm start and local alignment are cleared. If stopping is rejected
or cannot finish within stopping_horizon_steps, UnsafeStoppingTrajectory withholds authorization.
Invalid inputs, model failures and handshake failures also withhold authorization.
These predictive checks do not establish braking safety for uncalibrated actuators
or tire slip.

Every normal Brake/Hold/RequestMode is checked against fresh stopping
constraints before publication. This includes goal settling, yaw/mode-dwell waits,
external motion after completion and pending explicit handshakes. Successful normal
stops keep their navigation state and immutable request; they do not become
Waiting/Blocked. Rejection clears the request and emits UnsafeStoppingTrajectory,
so ModeExecutor cancels the transition and latches fault. Since 0.14.1 this check
uses ActuationModel::plan_stopping and RolloutEngine::generate_execution: the actual
first non-driving interval, including permitted stationary steering, followed by a
complete braking tail. Brake retains measured steering; Hold/RequestMode may align
only after complete proportional braking and only when initial feedback permits
handover. A pure brake trace cannot authorize a pending alignment against newly
changed hard constraints. The nominal check preserves actual mode, request ID and
unconfirmed feedback; it never grants drive permission or synthesizes an
acknowledgement. It does not preview protocol state; TimedExecutor still checks the
actual supervisor result at execution. Use plan_stopping for nominal prediction,
not as a substitute for the guarded execution entry point.

Version 0.10 also gates every first Drive plus its complete stopping continuation
through that validator. `stopping_horizon_steps` is an independent bounded budget
(default 200 ticks) for entry/alignment, first Drive and braking. A rejected Drive
falls back to the separately checked current stop; warm starts are accepted only
after a validated Drive. `generate_stop` ends when modeled wheels/body reach zero,
including motion below handover thresholds, and returns invalid on exhaustion.
Zero intent emits Brake/Hold with measured steering and zero drive targets, even
when measured wheel residuals exceed the Drive allowance.

Version 0.11 specifies Drive actuator interpolation: from the freshly measured
wheel speeds and steering angles, reach both supplied endpoint target arrays
with affine interpolation over the full dt_s. Reaching a deceleration endpoint
earlier and holding it does not implement this contract. The body_command is
endpoint forward kinematics, not the average velocity of the tick. Brake keeps
measured steering and scales measured wheel speeds proportionally to zero at the
fastest common body/joint braking rate, holding zero for any remaining time.
The adapter must implement and calibrate these profiles; the bounds do not cover
unspecified servo dynamics or tire slip.

In 0.12 Stable Drive also rejects fresh `mode_confirmed=false` feedback and a
measured actual mode different from the executor's stable mode. A pending mode
handshake still masks Drive and continues its committed transition.

The executor independently rejects absolute DualAckermann vx/yaw-rate, Crab
translation-speed and Spin yaw-rate violations. Transient manifold projection
tolerance cannot enlarge these limits. Drive interpolation must also obey
pointwise linear/angular and joint rates, including both sides of a reversal.
Absolute speed caps are certified over the whole joint interpolation, rather than
only at its endpoints. Analytic triangle bounds, adaptive chord enclosures and
derivative-sign bounds cover unsampled peaks. Certification is bounded to 4096
intervals and depth 14; exhaustion rejects the target. The model can reserve a
rolling-speed margin in 1% increments while preserving desired steering geometry,
with at most 60 target refinements. Measured overspeed may recover through the
checked Brake path.

Every first Drive and its complete stopping continuation still passes the shared
validator. A rejected candidate retries up to safety_reduction_attempts amplitude
halvings (default 8, maximum 16; zero disables retries). Each attempt revalidates
the entire continuation. Reductions preserve translation direction and Ackermann
curvature, frozen entry geometry and the original alignment deadline. Success
clears the optimizer warm start; exhaustion uses the separately checked current
stop. Output::safety_reductions counts reduced validations across all candidates
in one compute call, separately from optimization rollout statistics.

Drive trajectories integrate full-tick affine encoder targets with analytic yaw
and bounded midpoint translation quadrature. Brake uses proportional body ramps.
Both include conservative segment curve-to-chord enclosures and accumulated
position error. A caller-supplied trace with empty sweep_margins_m denotes
piecewise straight motion; provided margins must be finite, nonnegative and match
the pose segments.

All inclusive time boundaries share a 1 ns numerical floor, enlarged to four times double precision epsilon times
the timestamp magnitude for subtraction of large stamps. Deadline receipt at the
boundary is accepted; actual lateness beyond that tolerance faults. The same rule
covers alignment minima, dwell quantization, stopping/alignment prediction and
command/feedback age limits. Duration-to-ticks rounding removes numerical extra
ticks. Timestamp monotonicity, sequence and replay checks remain strict.

## Guarded timing and command envelopes

Use TimedExecutor for queued/transported commands. In 0.14 its update API takes the
current ControllerInput, including path, obstacles and vehicle state, rather than
VehicleState alone. Its validator may be the same shared const validator supplied
to Controller. The validator must have compatible safety limits.

CommandEnvelope requires a nonzero session_id, strictly increasing nonzero sequence,
strictly increasing issued_at_s, Output, source_stamp_s, execute_at_s, valid_until_s
and source_task captured from the same planning input.
The last three fields have invalid defaults: legacy envelopes fail closed.
source_stamp_s is the observation used to compute the plan; rewrapping/republishing
must not refresh it. execute_at_s is the earliest scheduled application instant;
valid_until_s is the final admissible start instant. Required ordering is
source_stamp_s <= issued_at_s <= execute_at_s <= valid_until_s. All times are finite,
nonnegative and in one clock domain. Request retries preserve their mode request
payload but use a new envelope sequence/issue time and truthful source metadata.
CommandTask::capture(planning_input) owns a value snapshot of path_id, heading_policy
and every ordered reference_path pose (x, y, yaw). These are the same exact task
identity fields used by PathManager. Geometry changes are detected even when path_id
is reused; an identical path with a new ID is a new task. Vehicle motion, obstacles
and Controller-generated tracking context do not change task identity.
Capture before queueing and preserve the snapshot during transport. Do not bind an
old Output to the latest task merely to make it pass: recompute for the new task.

```cpp
#include <swerve_mppi/planning/controller.hpp>
#include <swerve_mppi/execution/timing.hpp>

swerve_mppi::Controller controller(config, validator);
swerve_mppi::TimedExecutor executor(config, session_id, initial_actual_mode, {}, validator);
// planning_input is the actual observation used to compute this plan.
const auto output = controller.compute(planning_input);
swerve_mppi::CommandEnvelope envelope{
    session_id, ++sequence, issued_at_s, output,
    planning_input.vehicle.stamp_s, application_time_s, valid_until_s,
    swerve_mppi::CommandTask::capture(planning_input)};
// latest_input describes the application instant and current obstacles/constraints.
const auto guarded = executor.update(envelope, latest_input, application_time_s);
if (guarded.actuation) {
  const auto targets = guarded.actuation->sample(elapsed_since_application_s);
  // Apply targets at the actuator rate; convert wheel m/s to joint rad/s.
  // An absent sample is an expired/invalid interval, never a held Drive.
}
// Map guarded.execution.feedback to the next state observation.
// A missing-command watchdog tick calls update(std::nullopt, latest_input, now_s).
```

TimingLimits defaults remain max_feedback_age_s=0.15, max_command_age_s=0.15 and
period_tolerance_ratio=0.25. Tick and measurement stamp intervals must match dt_s
within that ratio. Observation, source-observation and command ages are bounded;
source age uses max_feedback_age_s. Inclusive boundaries use the shared numerical
time tolerance. Invalid source/schedule ordering reports InvalidPlanTime; an old
source reports SourceTimeout; early application reports NotYetExecutable; application
after valid_until_s reports ExecutionExpired. Existing clock, session, replay and
missing-command checks still latch SafeStop. The guard never repeats a previous Drive.

Freshness is not execution-state alignment. TimedExecutor additionally requires
latest_input.vehicle.stamp_s == now_s within numerical tolerance. A merely recent raw
observation reports ExecutionSafetyError::StateNotCurrent and latches SafeStop.
For stepped validation, take the snapshot at the application boundary. A real-time
adapter must supply a genuinely time-aligned state and account for prediction/measurement
uncertainty; rewriting a stale timestamp is not alignment. This stage implements no
latency predictor or uncertainty model. Current obstacles/constraints must also be
coherent with that snapshot; dynamic obstacle prediction is not implemented.

Before any protocol preview, TimedExecutor compares the required source_task with
the latest task. A missing snapshot or mismatch discards the queued Output and tries
the same independently checked stopping fallback described below. A healthy fallback
reports ExecutionSafetyError::TaskMismatch and rejected_status=Invalid; it never
authorizes the obsolete Drive or consumes an obsolete new mode request ID. The next
fresh command for the current task can recover without reset. If stopping is unsafe,
UnsafeStoppingTrajectory takes priority and SafeStop latches. Already committed
mode transitions retain their original protocol identity/deadline; task mismatch does
not introduce a cancellation operation. TimingGuard alone remains a transport guard
and cannot check task identity without current ControllerInput.

For a matching task, TimedExecutor compiles target_velocity with DriveModel at
the latest measured state, then previews ModeExecutor and builds an ActuationPlan
for its actual result. Frozen mode-entry joints are cached only on admission. RolloutEngine::generate_execution validates the
exact wheel/steering interval followed by a complete stop, using the latest context
and the same stopping_horizon_steps budget. After compilation it does not reinterpret
the joint-level body_command FK as a new ideal control; validation uses the exact
compiled joint targets. The public target_velocity is the original nominal intent.
A valid preview commits once. A rejected well-formed interval/continuation tries a
separate checked stopping preview. Success returns CommandRejected, the rejected
TrajectoryStatus and a healthy actuator profile; a later safe command may recover
without reset. A failed complete stop returns UnsafeStoppingTrajectory and latches
SafeStop. Invalid context/actuation also latches SafeStop. Protocol faults remain
faults; they are not turned into healthy braking. Rejected new mode requests do not
consume their request IDs or commit their transition. Report execution rejection using TimedExecutionResult; do not
announce completion from an obsolete planner Output after a rejection. Already committed transitions
retain their original deadline and identity during a healthy stopping fallback.

ActuationPlan is generated by ActuationModel and cannot be constructed with unchecked
public endpoint fields. sample(t) returns linear rolling wheel speeds and mechanical
steering angles for finite 0 <= t <= dt_s; it never extrapolates. Drive samples are
affine from measured joints to both endpoint arrays over the full tick. Brake keeps
measured steering and scales all wheel speeds proportionally. Its full braking time is
max(hypot(vx,vy)/max_linear_decel_mps2, abs(wz)/max_angular_decel_radps2,
max_i(abs(wheel_speed_i))/max_wheel_accel_mps2), with encoder-derived body velocity.
Hold/RequestMode may align only after this entire brake completes and only when the
initial handover threshold allows alignment. Steering then advances at the configured
rate during remaining stopped time. Plan/rollout actuator-model parameters must match.

These deceleration values define the required nominal actuator reference, not a
minimum guaranteed physical braking capability. Sending zero wheel speed immediately,
independent wheel ramps, early endpoint holds, or another mode-inference supervisor
implements a different plant. Calibrate servo tracking, achievable braking, delay and
slip against an independent plant before claiming physical stopping safety. This
stage supplies the exact reference profile, not that calibration or a robust
physical stopping envelope. SafeStop disables drive and has no certified normal
braking profile; physical emergency behavior belongs to the independent watchdog.

The adapter supplies the clock, serial ticks, feedback mapping and exclusive wheel
command ownership. A paused simulation clock does not replace an actuator watchdog;
the core cannot enforce stopping when its process is not called. A sampled profile
must start at its validated application instant and cannot be reused after the tick.

Recovery requires independently verified stopped/confirmed feedback, draining pending
commands, executor.reset(recovered, new_session_id) with a strictly larger session ID,
and controller.reset(). Clock/sequence may restart; request ID high-water marks remain.
Across process restarts the caller must persist/coordinate session identity and drain
transport. Planning/context snapshots and model validity remain the caller's contract.

## Regression boundaries

CTest's execution suite covers all six directed mode changes with an independent
rate-limited encoder fixture; frozen retries; stale acknowledgements; measured
steering/wheel gates; deadline latching; cancellation and ID replay after recovery;
mode persistence; valid entry geometry; request ID exhaustion; drive/encoder
consistency and steering tracking tolerance; backward clocks; curved Ackermann
and Spin progress; and controller to executor to measured-feedback lateral
progress. The fixture is deliberately small
and has no Gazebo, ROS, terrain, tire slip, communication transport or dynamics
calibration. Its results establish the core protocol behavior only.

## Same-mode steering commitment

A large steering change in the current mode produces Brake/Hold until it can
resume bounded Drive. Controller freezes the control intent across those cycles,
checks the continuation against fresh obstacles, and applies the original
confirmation_timeout_s bound. This does not issue a ModeRequest or reset mode
age. Reset or a new task path cancels obsolete local alignment. An explicit mode
request is preserved across replans; task cancellation still uses SafeStop/reset.

Tracking, corner and terminal policies share the same local commitment and timeout.
Changing policy cannot renew the deadline or discard an unfinished alignment.

Mode-entry intent is also retained for the first Drive after a confirmed switch,
so stochastic optimization cannot immediately undo the steering handshake.

## Navigation lifecycle in 0.5

Controller now owns ordered-path progress and measured goal completion. Navigation
status is separate from execution phase. Replanning clears local optimizer and
same-mode alignment state, while an active RequestMode retains its ID, complete
payload and original deadline. Completion requires stopped body/wheels and a
confirmed mode over a settling dwell; afterward Controller holds the task stopped.
See [NAVIGATION.md](NAVIGATION.md) for path identity, heading policy and diagnostics.
