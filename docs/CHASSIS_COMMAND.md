# Command and observation contract

`Controller::compute(const ControllerInput&)` returns `Output`: planning diagnostics
and `std::optional<ChassisCommand>`. The command holds an explicit mode, body-frame
`target_velocity` and an optional `ModeRequest{id, mode, entry_velocity}`. It contains
no joint targets, transport session, packet sequence or executor action.

| Result | Meaning |
| --- | --- |
| Present nonzero velocity | Nominal drive intent in the selected mode |
| Present zero, no request | Ordinary braking/holding in the actual mode |
| Present mode request | Zero drive with frozen entry intent; wait for measured confirmation |
| Absent command | No authorized command; external caller must handle stopping/faults and discard old output |

Entry velocity describes desired alignment geometry, never permission to drive.
Request IDs increase above the last observed/issued ID. Retries preserve ID, target
mode and entry velocity. A pending transition cannot be overwritten by a new plan.
Only a matching measured receipt, actual mode, stopped state and alignment can
complete the planner's request. Controller reset clears planning state but does not
reset or stop a physical chassis. Live code must use the guarded recovery API rather
than treating an unconditional offline reset as chassis recovery.

`VehicleState` contains pose, body twist, steering angles, linear rolling wheel
speeds, actual mode, confirmation/fault flags, last request ID, actual mode age and
optional `AcceptedModeRequest`. That receipt contains the original entry velocity
and the mechanical steering geometry accepted by the chassis. Wheel order is
FL, FR, RL, RR. Core wheel speeds are m/s; convert wire joint rad/s using wheel radius.

Pose, path and obstacles share one world frame; velocities use body x-forward,
y-left and positive yaw CCW. All source clocks use `TimestampNs` integer nanoseconds
in one caller-selected domain. Joint, pose and mode sources may differ only within
`observation_pairing_tolerance_s`; `FeedbackAdapter` retains the newest paired source
stamp and never rewrites it to the planning time.

`ControllerInput::planning_stamp_ns` is the decision-clock sample for the compute.
The vehicle observation must satisfy configured future, age and inter-observation-gap
bounds. `Output` separately carries observation, computation, initially-unset
publication and expiry stamps. Call `set_publication_stamp` exactly once immediately
before sending, and require `command_valid_at` at consumption. An expired output is
not a stop command and must never be reused.

On the next input, `CommandApplication` identifies the preceding command and reports
its publication plus earliest/latest application stamps. Exact bounded history is
propagated and checked against measured joints. A missing report or nonzero admissible
window is diagnosed and cold-seeded; invalid identity/time or excessive uncertainty
is a fault. Publication/application stamps are evidence, not replacements for the
source observation stamp.

`MotionPolicy::EncoderNominal` explicitly selects encoder-only nominal planning.
`RequireIndependent` requires a fresh `IndependentBody` observation with deterministic
linear/angular error bounds. Admitted residual uncertainty expands collision margins;
an exceeded envelope or hidden body motion while encoders report stopped withholds
authorization.

Clock, feedback, motion, compute and transition failures latch. `Controller::recover`
requires a newer coherent observation; `reset` is an unconditional offline restart.
`planning_period_s` controls the compute budget, `model_period_s` the rollout grid and
`chassis_period_s` the command-mechanics substeps.

`current_chassis::Adapter` retains the chassis's accepted request ID and entry
velocity on ordinary drive/hold packets, pairs state with odometry and rejects stale
results. It accepts explicit publication/application evidence; it never derives an
exact application time from a state message that lacks MPPI command identity. These
wire responsibilities remain outside `Controller`. The algorithm may propose a mode
change; actual braking, steering and confirmation belong to the chassis. No joint
execution API is installed by this package.
