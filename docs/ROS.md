# ROS operation and configuration

Baseline: Jazzy / Ubuntu 24.04 with Harmonic. `controller.launch.py` starts a regular
node; no lifecycle calls or execution evidence producer is required. `demo.launch.py`
also starts the companion chassis. The optional `namespace` argument belongs to
`controller.launch.py`; configure frames/topics explicitly for prefixed robots.

| Interface | Type | Purpose |
| --- | --- | --- |
| `path` | `nav_msgs/Path` | New ordered planar path; each delivery starts a task |
| `odom` | `nav_msgs/Odometry` | Pose and measured body velocity |
| `chassis_controller/state` | companion `ChassisState` | Actual/target modes and measured joints |
| `costmap` | `nav_msgs/OccupancyGrid` | Optional grid for the ROS-side critic |
| `chassis_controller/command` | companion `ChassisCommand` | Adapter-translated body velocity/mode intent |
| `~/goal_reached` | `std_msgs/Bool` | Transient-local task completion; false on a new path |
| `~/prediction` | `nav_msgs/Path` | Selected nominal prediction when subscribed |
| `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Compute duration, evaluated candidates, cost and mode |

Path subscription is reliable/volatile depth one, compatible with standard Path
publishers. Send after discovery. State uses sensor QoS and odometry uses reliable
depth ten. Costmap uses reliable/transient-local depth one. Publish a full static
grid from a transient-local publisher. Map coordinates must already be in `odom_frame`
with an axis-aligned origin. Path TF is resolved when the task is received; resend
the path to reflect later global localization corrections. Mixed path frames are
rejected. Poses require planar normalized quaternions.

All settings are startup configuration. Defaults live in core `Config` and ROS
`controller.yaml`. Geometry/actuator parameters must describe the chassis; they
are explicit configuration, not remotely queried startup gates. Units are meters,
seconds and radians. Core wheel speeds are rolling m/s; the adapter converts the
companion's encoder rad/s using `wheel_radius`.

| Parameter group | Names |
| --- | --- |
| Timing | `control_frequency`, `state_timeout`, `model_dt`, `integration_dt` |
| MPPI | `horizon`, `samples`, `iterations`, `seed`, `temperature`, `noise_linear`, `noise_angular`, `noise_correlation` |
| Geometry | `wheelbase`, `track`, `wheel_radius`, `min_turn_radius`, `steer_limit` |
| Response | `max_speed`, `max_yaw_rate`, `linear_accel`, `angular_accel`, `max_wheel_speed`, `wheel_accel`, `steer_rate` |
| Alignment | `realign_angle`, `alignment_tolerance`, `alignment_dwell`, `stopped_wheel_speed` |
| Mode selection | `minimum_mode_dwell`, `switch_penalty`, `switch_hysteresis` |
| Tracking | `lookahead`, `approach_distance`, `approach_gain` |
| Critics | `path_distance_weight`, `path_heading_weight`, `progress_weight`, `goal_position_weight`, `goal_heading_weight`, `effort_weight`, `smoothness_weight`, `steering_change_weight` |
| GoalChecker | `goal_position_tolerance`, `goal_yaw_tolerance`, `goal_linear_speed`, `goal_angular_speed`, `goal_settle_time` |
| Map critic | `use_costmap`, `costmap_weight`, `unknown_is_obstacle` |

`state_timeout` checks source age and wall silence. Missing observations suspend
normal publication and clear warm-start; the chassis owns command timeout behavior.
A failed compute produces a current-mode zero. No runtime fault latch prevents the
next valid observation from being used. This is a nominal path follower; no independent
emergency-stop or physical braking guarantee is provided by the MPPI package.

Stage one uses `use_costmap=false`. With `use_costmap=true`, missing maps produce
NoSolution, unknown/off-map/lethal cells can reject candidates, and nonlethal costs
are accumulated. Inflate the supplied map for the robot radius before enabling this
center-point critic. There is no separate circle-obstacle message pipeline.
