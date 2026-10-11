# ROS integration (stage 5B)

The supported build/test baseline is ROS 2 Jazzy on Ubuntu 24.04. This stage supplies
transport, lifecycle and publication gates. It does not claim calibrated physical
prediction or a completed Gazebo closed loop. The core stays at version 0.26.0;
the new ROS package/repository release is 0.27.0. No legacy topic or executor aliases
are installed. The Gazebo repository is unchanged.

## Workspace build

Place both repositories in a colcon workspace source directory. The reviewed chassis
contract is `zz990099/swerve_gazebo_sim` commit
`9358c96884d7fa4866cea7c053daa7a7cec82d8f` (0.8.0).

```bash
source /opt/ros/jazzy/setup.bash
# Build the companion messages/Python package without simulator runtime dependencies.
colcon build --packages-select swerve_gazebo_sim --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
colcon build --packages-up-to swerve_mppi_ros --packages-skip swerve_gazebo_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select swerve_mppi_ros --return-code-on-test-failure
colcon test-result --all --verbose
source install/local_setup.bash
ros2 launch swerve_mppi_ros controller.launch.py
```

The simulator itself and its joint controllers are needed for stage 5C, not for
generating chassis messages or running these synthetic transport regressions.
The ROS package uses `find_package(swerve_mppi_core 0.26 CONFIG REQUIRED)` and links
the installed target. It never includes private planner files or builds core sources.

## Startup and lifecycle

Start in the same namespace as `swerve_controller`, or configure explicit topic/node
names. `use_sim_time=true` is required. Configure the node, allow it to query the
chassis's actual shared parameters asynchronously, and supply a path and fresh obstacle
snapshot. Activation requires a positive advancing simulation clock and a fresh,
coherent, stopped, ready and confirmed chassis observation.

```bash
ros2 lifecycle set /mppi_controller configure
# Wait for verified chassis parameters and supply task/obstacle inputs.
ros2 lifecycle set /mppi_controller activate
```

`planner_profile` is an optional bounded core `name=value` profile file; an empty
path uses validated core defaults. Planner policies are not copied from Gazebo YAML.
The wrapper queries chassis geometry, rate, velocity/acceleration/steering limits,
transition rules, command timeout and odometry/frame settings. Shared numerical
mismatches prevent activation; failed queries are retried while inactive. Snapshot
parameters belong to startup, matching the chassis's constructor-time configuration.
Restart/reconfigure both nodes after changing chassis mechanics.

All wrapper parameters are read-only after construction; adjust YAML/launch overrides
and restart to change them. Changing away from simulation time while configured is
rejected. The provided executable uses a single-threaded executor and serialized
callbacks. Use the same serialization when embedding `ControllerNode`, with Jazzy's
dedicated `use_clock_thread=true` option so `/clock` can advance during compute.
Clock-jump callbacks retain an atomic rollback latch even if time advances again
before the main executor observes it. Publication rereads the ROS clock after compute.

## Interfaces

All non-private topic defaults are relative to the controller namespace. No TF lookup
or automatic frame conversion is performed. Path/obstacle coordinates are in the
configured odometry frame; velocities are in the body frame.

| Interface | ROS type | Contract |
| --- | --- | --- |
| `chassis_controller/state` | `swerve_gazebo_sim/ChassisState` | Best effort sensor QoS; original encoder source stamp |
| `odom` | `nav_msgs/Odometry` | Reliable depth 10; same encoder observation, configured body/odom frames |
| `path` | `nav_msgs/Path` | Reliable volatile depth 1; complete nonempty bounded task, planar unit-quaternion poses; each delivery starts a task |
| `obstacles` | `swerve_mppi_ros/ObstacleArray` | Reliable volatile depth 1; strictly increasing source stamps, finite circles, fresh snapshot; explicit empty means no observed obstacles |
| `chassis_controller/command` | `swerve_gazebo_sim/ChassisCommand` | Reliable volatile depth 1; adapter-authorized body velocity/mode request only |
| `~/command_publication` | `swerve_mppi_ros/CommandPublication` | Exact issued wire command plus MPPI ID and observation/compute/publication/expiry timestamps |
| `~/report_application` | `swerve_mppi_ros/ReportApplication` service | External execution window for the pending publication; rejects wrong ID/publication identity, future, reversed, expired or over-wide windows |
| `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Active publication/evidence/fault status; core/adapter error codes refer to public enums |
| `~/recover_chassis` | `std_srvs/Trigger` service | Explicit fault-only zero recovery, immutable retries; does not clear the planner/runtime fault |

Pose conversion rejects vertical displacement, roll/pitch, nonfinite values and
nonunit quaternions. State/odom queues each retain at most 32 source samples. Pairing
consumes both sources once, within the core's pairing bound, preserving their times.
Identical duplicates are ignored; conflicting equal-stamp snapshots fault. Delayed
samples older than consumed data are dropped and never become new observations.
Mode receipts and raw wheel angular velocities are copied without reinterpretation;
the existing core adapter validates them and converts wheel units.

## Publication and application evidence

The first computation uses a stopped startup snapshot. Each valid output publishes
once, before expiry. Planning then waits for externally supplied application evidence
and an observation whose source time is no earlier than the evidence window's end.
`command_id` is scoped to one configured lifecycle; the service also requires the exact
`published_stamp` to reject evidence from an earlier lifecycle with a reused ID.
The evidence service consumes each pending identity once. Publication metadata is
not an execution acknowledgement, and `ChassisState.request_id` is a mode receipt,
not an MPPI command acknowledgement.

An exact window is appropriate only for an instrumented executor with observed
application time. A bounded window requires a measured, justified delay contract.
There is no default guessed delay or test evidence producer in the production launch.
If no producer is connected, only the first command can publish; its expiry then
latches the wrapper. Stage 5C must establish a physically defensible evidence source
or calibrated bounded contract in the algorithm/integration side, keeping MPPI IDs,
tasks and planning out of the chassis repository.

## Stop and recovery

Expired/missing observations or obstacles, invalid input, missing application evidence,
core/adapter rejection, clock pause (wall timeout) or rollback latch normal publication.
The node never republishes an old drive. Deactivation and faults stop sending; the
chassis owns timeout and stopping. This is a watchdog stop, not an immediate emergency
stop: its latency includes the chassis command/wall timeout and physical deceleration.

For a chassis fault with fresh coherent feedback and an advancing clock, call
`~/recover_chassis`. It sends a zero request above the observed request high-water
mark and retries the same ID/entry intent until confirmed. These are chassis recovery
commands, not planner outputs, and produce no fabricated MPPI application metadata.
After stopped confirmation, deactivate, cleanup, configure and activate deliberately.
Cleanup discards old buffers/history; reconfiguration observes the chassis's retained
request identity before planning. Ordinary deactivate/activate alone is rejected after
the first activation, including after clock rollback. There is no live `Controller::reset`.

## Validation and next stage

Native Jazzy tests use a synthetic stopped chassis and real ROS publishers/services.
They cover conversion/mode fields, pairing order/duplicates/bounds, parameter and
activation gates, publication identity, evidence admission, missing evidence, clock
pause/rollback, observation/obstacle expiry, deactivation and immutable zero recovery.
The stopped fixtures give exact application evidence only for synthetic zero commands;
they do not validate a moving physical plant. CI builds the companion contract and
both algorithm packages, then runs the ROS tests. Core Debug/Release, offline parity,
UBSan and standalone installed-consumer checks remain independent jobs.

Stage 5C: implement measured execution evidence; calibrate delay, steering/wheel
response and stopping; validate zero/recovery, straight/crab/spin and all transitions;
then validate paths/obstacles/fault injection with quantitative trace thresholds.
Stage 6: a separate Nav2 package maps lifecycle, path and costmap inputs only after
the standalone physical acceptance passes.
