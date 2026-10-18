# Nav2 controller integration

ROS package version **0.29.0** targets **ROS 2 Jazzy / Nav2 Jazzy** on Ubuntu 24.04.
The numerical core remains independently installable at 0.28.0. Humble and Rolling
have different controller APIs and are not claimed as tested targets.

## Plugins and ownership

`swerve_mppi_ros::Nav2Controller` implements `nav2_core::Controller` and is exported
through pluginlib. It implements configure/activate/deactivate/cleanup, setPlan,
computeVelocityCommands, setSpeedLimit, cancel and reset. Nav2 owns execution rate,
robot pose/velocity, the local costmap, FollowPath actions and task completion.
There is no independent path subscription or controller timer.

`swerve_mppi_ros::OrderedGoalChecker` implements `nav2_core::GoalChecker`. Use it
with this controller: it requires final ordered-segment progress, settled position,
yaw and measured speed, and no pending mode transition. It starts closed on every
new action because Nav2 checks completion before its first controller call. This
prevents a closed path from succeeding at its initial pose. Standard Nav2 goal
checkers do not carry this ordered-progress contract.

`setPlan` validates and stores the source-frame path. On every compute, robot pose
is transformed into that frame to preserve ordered progress; the local section and
endpoint are transformed into the costmap global frame using current TF. Mixed
path frames and invalid planar quaternions are rejected. A local section excludes
later return legs. Passing an endpoint also requires proximity to its segment,
so an off-path projection cannot advance progress by itself.

## Velocity and mode routing

Nav2 returns only `TwistStamped`; the chassis also needs an explicit mode and
frozen transition entry. The plugin therefore publishes a companion
`ChassisCommand` envelope on `mppi/command_context`. This is context for the bridge,
never a second drive output. The plugin preserves request IDs/entry geometry and
returns zero throughout an unconfirmed transition.

The `chassis_bridge` executable combines this context with Nav2's **final**
`cmd_vel`, then publishes `chassis_controller/command`. Context alone cannot drive.
New request IDs always carry zero drive velocity. Nonzero drive requires fresh
context/velocity/state and confirmed matching mode, request ID and entry geometry.
Nav2 zeros, stale inputs and zero contexts hold zero. With stale chassis feedback,
the bridge stops renewing commands and the chassis watchdog applies its stop.

Use exactly one bridge/command-topic owner. If adding a velocity smoother or
collision monitor, remap the bridge input to that pipeline's final output. All
processors must preserve the selected mode's velocity constraints; for example,
Spin must not contain residual linear velocity. The supplied initial test launch
uses a direct controller-server-to-bridge connection.

| Interface | Type | Owner |
| --- | --- | --- |
| `follow_path` | `nav2_msgs/action/FollowPath` | Nav2 controller server |
| `odom` and TF | Normal Nav2 inputs | Controller server / local costmap |
| `chassis_controller/state` | Companion `ChassisState` | Plugin and bridge observations |
| `cmd_vel` | `geometry_msgs/TwistStamped` | Nav2 final speed output |
| `mppi/command_context` | Companion `ChassisCommand` | Plugin mode/request context |
| `chassis_controller/command` | Companion `ChassisCommand` | Bridge final output |
| `FollowPath/prediction` | `nav_msgs/Path` | Plugin nominal prediction |
| `FollowPath/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Plugin timing/cost |

The old `/path`, `~/goal_reached` and standalone `controller` executable are
removed. FollowPath results replace the old completion topic.

## Configuration

`config/controller.yaml` is a minimal **FollowPath-only**, obstacle-free test setup.
It starts neither a global planner, localization nor BT navigator. The local
costmap is rolling in `odom` with an inflation layer and free initial cells. Add
static/observation layers and robot-appropriate inflation before obstacle testing.
The center-point cost critic consumes a locked, immutable Nav2 costmap snapshot;
it is not a swept-footprint collision validator or completed obstacle-navigation
acceptance.

In an existing Nav2 configuration add:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: [FollowPath]
    goal_checker_plugins: [goal_checker]
    enable_stamped_cmd_vel: true
    FollowPath:
      plugin: swerve_mppi_ros::Nav2Controller
      state_topic: chassis_controller/state
      context_topic: mppi/command_context
      use_costmap: true
    goal_checker:
      plugin: swerve_mppi_ros::OrderedGoalChecker
      xy_goal_tolerance: 0.06
      yaw_goal_tolerance: 0.05
      trans_stopped_velocity: 0.035
      rot_stopped_velocity: 0.035
      settle_time: 0.3
```

Launch only the bridge alongside an existing controller server; do not launch a
second controller server. Set bridge frames, topics and `enable_stamped_cmd_vel`
to match the host Nav2 configuration. The bridge also supports unstamped Twist
when explicitly configured, using reception time for freshness.

Core parameters are under `FollowPath.*`: geometry, response, horizon/samples,
noise, tracking weights, mode dwell/switch cost and approach settings retain the
names in core Config and TrackingWeights. Extra plugin settings are
`state_timeout`, `transform_tolerance`, `state_topic`, `context_topic`,
`use_costmap`, `costmap_weight` and `unknown_is_obstacle`. Parameters are startup
configuration. `setSpeedLimit` supports absolute m/s and percentages of the
configured maximum; Nav2's zero `NO_SPEED_LIMIT` sentinel restores the baseline.
Angular and wheel limits remain enforced independently.

The bridge parameters are `body_frame`, `state_topic`, `context_topic`,
`cmd_vel_topic`, `command_topic`, `enable_stamped_cmd_vel`, `command_timeout` and
`publish_frequency`. Core wheel speed is rolling m/s; companion feedback is rad/s
and is converted with `FollowPath.wheel_radius`. Geometry and response settings
must match the chassis. For namespaces, provide a correspondingly scoped Nav2
parameter file and explicit frame names.
