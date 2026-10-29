# Map-path validation with ideal external localization

The 32-case benchmark defaults to one round, fixed seed 42 and `--path-frame map`. Each case
starts a fresh Gazebo world, keeps raw wheel `/odom` and its TF, and enables
the chassis repository's optional
`publish_ground_truth_map_tf` node. No controller, critic, actuator, or wheel
odometry algorithm is changed by this experiment.

The fixed reference path is generated once in the Gazebo truth reference,
which defines `map`. The local costmap retains `global_frame: odom`; the MPPI
plugin transforms the map path and goal into that frame on every control step.
There is no planner, SLAM or mapping pipeline in this FollowPath benchmark.
The ideal external localization TF replaces SLAM's pose observation only.

```bash
python3 swerve_mppi_ros/tests/run_trajectory_rounds.py \
  --chassis-root companion-chassis --output artifacts \
  --rounds 1 --seed-base 42 --min-path-length 12 --max-speed 1.0 --path-frame map
```

The benchmark requests a nonzero spawn pose `(2, -1, 0.4 rad)` to exercise
translation/rotation between the fixed reference and initial wheel odometry.
The map/truth reference is the one supplied by Gazebo's OdometryPublisher;
it is not assumed to be interchangeable with an arbitrary external world frame.

`result.json` records the actual map path, initial truth/wheel origins, raw wheel
odometry source and ideal localization source. `map_to_odom.csv` records TF
source stamps and planar corrections; `wheel_odometry.csv` records raw wheel
poses. `trajectory.csv` retains separately source-stamped truth and wheel
samples. Plot coordinates are a *fixed* rotation/translation of map aligned
with the initial body, never a moving alignment. Error and average speed are
computed from truth; colour speed uses 0.2 s centred secants. The reference
length is 12 m except the in-place rotation case, and both planner and physical
speed caps are 1 m/s.

Acceptance remains completion plus peak deviation <= 10 cm, endpoint <= 20 cm,
and final yaw <= 5 degrees. Engineering limits (timing, slew, stopping) are
reported separately in JSON. Failed or incomplete cases remain in the montage.
This is ideal-localization controller validation, not a claim about real SLAM.
`--path-frame odom` uses raw wheel odometry without external localization,
for comparison with the map-path experiment. MPPI does not estimate odometry;
the chassis/localization layer owns `/odom` and its TF.
