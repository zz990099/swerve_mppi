# Simulation integration preparation (historical 0.20 reference)

This release supplies the feedback boundary, profile-consumer regressions and a
production-budget measurement tool, plus an actionless chassis command boundary. The separate Gazebo package supplies an
independent guarded ros2_control endpoint. The companion repository now also
supplies a ROS MPPI planning node, explicit execution controller and nominal
physical closed-loop regressions. The instructions below retain the original
preparation reference; they are not a current deployment acceptance report.
See [PREPARATION_PLAN.md](PREPARATION_PLAN.md) for the active core-only scope and
[CONFIGURATION.md](CONFIGURATION.md) for the new startup configuration contract.

Version 0.21.4 completes the active portable preparation series. Use
[ADAPTER_READINESS.md](ADAPTER_READINESS.md) and
[ADAPTER_ACCEPTANCE.md](ADAPTER_ACCEPTANCE.md) for complete peer profiles,
frame/clock identity, immediate external-fault revocation and supervised recovery.
These APIs do not retrofit the unchanged companion adapter automatically.

## Coherent feedback and scheduling

For ROS ingress use `FeedbackAdapter::make_at_nanoseconds`: retain the original
integer `stamp_ns` in JointObservation and StampedPose, and pass the original
integer application stamp. It compares all three exactly **before** converting
once to core seconds. Adjacent nanoseconds are rejected even at large epochs where
their double representations coincide. Missing/negative integer stamps are invalid;
the legacy `stamp_s` fields are ignored by this entry point.

The portable `make` seconds entry point remains available for non-ROS callers.
It permits only four scaled machine epsilons of conversion roundoff, without a
fixed nanosecond or freshness allowance. It cannot prove nanosecond identity at
large epochs; do not use it as the ROS synchronization boundary.

Both entry points accept one complete named encoder observation, one pose,
executor-owned ModeFeedback and the application stamp. They reject mismatched
stamps, partial or duplicate required joints, more
than 64 joint names, nonfinite data and inadmissible nominal feedback. Joint order
may vary; required names are `<prefix>{fl,fr,rl,rr}_{steering,wheel}_joint`.
It converts wheel rad/s into linear m/s and computes body-frame FK from that same
sample. Pose must already be expressed in the path/obstacle world frame. The ROS
owner must check frame IDs before conversion; the portable adapter has no TF tree.
The helper never combines separate odometry twist with encoders or rewrites a
stale measurement stamp. Encoder FK agrees by construction; it does not establish
physical no-slip motion.

Version 0.20 requires the executor-owned `accepted_mode_request` receipt to be
forwarded with mode status. `FeedbackAdapter` preserves it automatically. A manual
ROS conversion must carry its presence, request ID, mode, four mechanical steering
positions and the original body entry velocity. Keep the receipt through confirmation
and normal driving; clear it only on deliberate executor recovery or replace it
when a new request is accepted. Do not reconstruct it from current encoders or
planner predictions. Rebuild downstream consumers because the feedback/state
aggregate layout changed. The MPPI output remains velocity plus mode request.

The source-stamp equality check deliberately does not solve asynchronous transport latency.
For the first controlled test, pause/step the simulator at model boundaries and
collect a coherent snapshot, run planning with the physics paused, validate/install
at that boundary, then execute the next 0.1 s interval while sampling at >=100 Hz.
Keep the actuator callback separate from planning. Account for wall watchdog time
while paused: either complete planning before expiry or recover from a stop with
a new session. A normal continuously running ROS loop must implement and validate
an explicit timestamped estimator/prediction pipeline and its uncertainty bounds
before using a future execution-start state. Merely calling a snapshot "current"
is insufficient, even if it is younger than max_feedback_age_s.

Bind queued output to `CommandTask::capture` of its originating input and preserve
source/application/deadline metadata. TimedExecutor rechecks against the actual
execution-start snapshot and current obstacles. Publish ONLY ProfileRunner samples
from a checked result. No sample, SafeStop, late compute or failed admission must
stop refreshing the protected endpoint, causing its independent emergency stop.
Never wrap an old profile in a fresh source stamp.

## Chassis command boundary

Controller returns `Output.command` with body target velocity and an optional
immutable mode request; it publishes no joint angles, wheel speeds or action.
Keep the complete command together with its envelope metadata and source task.
TimedExecutor belongs to the lower chassis execution adapter: it compiles the body
target using the shared DriveModel and actual encoders, validates the exact joint
profile, and acknowledges the mode from measurements. Zero velocity cannot encode
a mode request or fault; absent authorization must stop/latch rather than retain an
old velocity. See [CHASSIS_COMMAND.md](CHASSIS_COMMAND.md).

The protected Gazebo endpoint remains the final joint actuator boundary. It consumes
ProfileRunner samples produced after body-command admission; do not send nominal
body targets or frozen mode-entry intent directly to its joint packet. Existing
Gazebo Twist control is not the complete timed/task/mode-acknowledgement protocol.
The companion now supplies the ROS chassis-command message and adapter. The
original 0.20 release changed the portable core/execution boundary.

## Execution endpoint

With Gazebo package 0.2 `external_joint_control:=true` starts one
`swerve_gazebo_sim/GuardedJointController`, replacing both forward controllers.
It claims all four position and all four velocity command interfaces exclusively.
Its subscription is `guarded_joint_controller/commands`; use the fixed v1 packet
and lifecycle described in that repository's `docs/EXTERNAL_ENDPOINT.md`.
The endpoint checks short simulation-time validity, sequence/session order and a
steady-clock watchdog in the controller update, independently of the MPPI process.
Faults command zero wheel speed and hold bounded measured steering. This emergency
response has no normal MPPI braking/collision certificate.

Startup/recovery order:

1. Independently verify measured stopped wheels/body, valid pose and confirmed mode.
2. Reset Controller, TimedExecutor with a strictly newer session, and ProfileRunner.
   Drain stale queues. Endpoint session numbers must increase across sender restarts.
3. Send a zero-wheel arm packet with measured steering and a new endpoint session.
   Retransmit that identical frozen arm while awaiting a matching healthy status;
   retries cannot renew deadlines. Then stream fresh targets.
   Recovery acknowledgement alone does not authorize Drive; core validation still does.
4. Install a checked profile at its application boundary, sample at actuator rate,
   and publish each sample with a short validity window. Preserve exclusive ownership.
5. On shutdown, send a fresh zero-wheel command while monitoring a measured stop,
   then stop publishing. Unexpected process death is handled by the endpoint.

Geometry mapping: wheelbase .60 m, track .50 m, radius .10 m; pi/2 steering stops;
2.5 rad/s steering rate; core wheel limit 2 m/s becomes 20 rad/s; core wheel
acceleration 4 m/s² becomes 40 rad/s². Body acceleration/deceleration limits remain
core assumptions, not independently enforced or certified by ros2_control.

## Reproducible checks

CTest adds `integration_regressions` and `profile_behavior_<scenario>` for all 14
existing scenarios and five seeds each. Those loops consume actual checked
TimedExecutor plans through ProfileRunner, integrate ONLY sampled joint targets
using independent encoder FK, and return named feedback through FeedbackAdapter.
They check completion/settling, mode confirmation, rate limits and measured obstacle
clearance at every actuator substep. Ideal servo tracking and midpoint integration
are fixture assumptions, not measurements of Gazebo contact dynamics.

With benchmarks enabled, run:

```bash
./build/swerve_mppi_integration_budget 50 > integration-budget.csv
```

Each repetition uses fresh instances and the same stationary curved-path snapshot
with 0/40/128 obstacles. It leaves the default 80 ms planning budget enabled and
measures compute + envelope capture + execution admission + initial profile sample.
CSV reports p50/p95/p99/max, minimum budget headroom, compute timeouts, total pipeline
overruns and other failures. Timing values are host measurements, not CI thresholds.
The tool does not include ROS/DDS or Gazebo CPU contention; repeat on the target
host with Gazebo running, then instrument the live adapter for state age, actual
application delay, missed actuator samples and endpoint faults. Do not infer a
production scheduling guarantee from an idle-host maximum.

Before accepting MPPI collision guarantees, compare commanded profile against
measured steering/wheel trajectories and independent Gazebo pose/twist for straight,
reverse, crab, spin, transitions and stopping. Measure latency, slip and worst-case
braking, update conservative model bounds, and repeat obstacles and fault injection.
The endpoint's publisher-loss smoke test proves a low-speed stop/recovery mechanism;
it does not close these physical acceptance items.
