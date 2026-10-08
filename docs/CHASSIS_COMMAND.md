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
recover, reset or stop a physical chassis.

`VehicleState` contains pose, body twist, steering angles, linear rolling wheel
speeds, actual mode, confirmation/fault flags, last request ID, actual mode age and
optional `AcceptedModeRequest`. That receipt contains the original entry velocity
and the mechanical steering geometry accepted by the chassis. Wheel order is
FL, FR, RL, RR. Core wheel speeds are m/s; convert wire joint rad/s using wheel radius.

Pose, path and obstacles share one world frame; velocities use body x-forward,
y-left and positive yaw CCW. Maintain path identity/geometry between calls and
supply fresh measured state. The current core expects one compute per `dt_s` and
strictly advancing state time. Observation/application synchronization and control
period decoupling are stage-3 work.

A future current-interface adapter must retain the chassis's accepted request ID
and entry velocity on ordinary drive/hold packets, pair state with odometry, and
reject stale results. These wire responsibilities are not implemented by Controller.
The algorithm may propose a mode change; actual braking, steering and confirmation
belong to the chassis. No joint execution API is installed by this package.
