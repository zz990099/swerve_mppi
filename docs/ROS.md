# ROS / Nav2 integration v1

The supplied configuration targets Jazzy. `swerve_mppi_ros::Nav2Controller` and
`swerve_mppi_ros::OrderedGoalChecker` load in Nav2 controller_server. The previous
standalone chassis bridge, command context, mode/request IDs and confirmation
protocol have been removed.

## Topics

- Input `chassis_controller/state`: `swerve_chassis_msgs/ChassisState`, sensor-data QoS.
- Output: standard Nav2 `cmd_vel` (`TwistStamped` with body frame and source stamp).
- Diagnostics: `FollowPath/diagnostics`, including compute_ms, cost, evaluated count and effective_samples.
- Prediction: `FollowPath/prediction`, published when subscribed.
- Localization/odometry and costmap remain ordinary Nav2 inputs.

Configure one final chassis command type. Do not run the old bridge. Keep
`enable_stamped_cmd_vel: true` for the bundled Jazzy setup; namespace and body
frame must match the chassis. Cancellation and zero publication remain Nav2's
responsibility; the chassis independently checks simulation and steady timeouts.

## Physical contract and tuning

`wheelbase`, `track_width`, `wheel_radius`, `physical_linear_speed`,
`physical_angular_speed`, `linear_accel`, `angular_accel`, `max_wheel_speed`,
`wheel_accel`, `steer_rate`, `steer_limit`, `steering_tracking_tolerance` and
`stopped_wheel_speed` must exactly match the chassis configuration (within numerical
tolerance). Wheels are FL/FR/RL/RR; angular units are rad and rad/s. These are
startup-only parameters. A mismatch rejects state rather than running two models.

`max_speed` and `max_yaw_rate` are separate planner caps. Nav2 speed-limit changes
rebuild the optimizer and clear warm start; they do not change physical limits.
Nav2's 0 absolute limit is its NO_SPEED_LIMIT sentinel; 0% explicitly stops
translation. Core permits a zero translation cap and still permits rotation.

Optimizer parameters: horizon, samples, iterations, model_dt, integration_dt,
noise_linear, noise_angular, noise_correlation, temperature and seed. Default
model_dt=0.1 s, integration_dt=0.01 s, horizon=24, samples=24. Confirm percentile
latency on the deployment CPU before choosing a controller frequency.
`position_scale` (0.03 m) and `heading_scale` (0.05 rad) normalize tracking
errors; `path_corridor` (0.04 m) and `heading_corridor` (0.10 rad) add continuous
slack penalties for departures without removing recovery candidates.
`cross_gain` defaults to 4/s. Path arc length drives terminal activation and speed
preview. Polyline vertices above 15 degrees are approached at zero speed; smooth
curves use lateral-acceleration and steering-servo tracking limits. Accepted
requested velocity is part of State, not substituted with measured body velocity.
`max_residual` is the hard predicted/measured wheel-vector disagreement limit in
m/s, not a permission to accept arbitrary slipping wheel commands.
`steering_time_constant` defaults to 0.10 s. `reference_velocity_weight` and
`presteering_alignment_weight` each default to 1.0. The reference-speed envelope
accounts for coupled wheel steering and drive demands. Invalid input, actuator,
collision and custom-critic rejection counts are published on diagnostics,
including computes that return NoSolution.

Goal checker state_topic/state_timeout can be set independently of the controller
but must identify the same robot. Feedback must be fresh and fault-free with all
wheel speeds <=0.05 rad/s, in addition to Nav2 pose and body-speed tolerances.

The default costmap is obstacle-free for physical tracking acceptance. Supply
real sensor/static layers for navigation. The collision critic uses the costmap's
circumscribed footprint radius, conservatively swept between prediction samples.

## Geometric heading policy

Intermediate path orientations and coincident-position yaw markers are ignored.
The final original pose yaw remains the task target. `lookahead` (0.30 m) sets
the geometric preview; `goal_heading_start_distance` (0.50 m) and
`goal_heading_full_distance` (0.10 m) define measured terminal blending.
`goal_heading_scale` (0.35 rad) normalizes the goal objective without changing
GoalChecker tolerances. `goal_heading_weight` is published with diagnostics.
See GEOMETRIC_HEADING.md for internal reference and retiming semantics.
