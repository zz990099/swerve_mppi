# Continuous MPPI validation

Old baseline: MPPI `7a1f912a557410056bcfea9d34d2a704e2b563ae` with chassis
`9358c96884d7fa4866cea7c053daa7a7cec82d8f`. New chassis pin is in dependencies.repos.
No old/new physical performance improvement is claimed without matched recordings.

## Automated layers

1. Standalone core: shared next-step target equivalence, continuous translation
   plus rotation, lateral presteering, invalid snapshots, invalid input, injected
   no-solution critics, zero planner limit and installed CMake consumer.
2. Portable caller contracts: ordered progress, closed/out-and-back path handling,
   stopped goal checks, grid bounds, footprint edge and between-sample collisions.
3. Portable closed loops: straight, lateral, reverse, spin, combined movement,
   S curve and return path, with execution steering rate reduced to 80% of nominal.
   These are numerical model tests, not independent physical evidence.
4. ROS plugin tests: plugin loading/cleanup, lifecycle, speed caps, continuous
   output, fault zero/recovery, physical configuration mismatch, TF/map errors,
   stale input and ordered goal completion.
5. Gazebo/Nav2 FollowPath: eleven independent physical scenarios including combined
   vx/vy/wz, fixed-heading S curve and live action cancellation, measured by independent Gazebo poses.
   Terminal gates are 0.15 m and 0.10 rad; ordered checkpoints are required for
   return/closed paths. Each case logs path RMS/peak, stopped sample fraction,
   wall duration and compute percentile diagnostics.

Local host has no ROS/Gazebo. ROS and physical acceptance must be established by
remote CI, not inferred from portable prediction tests. CI uses Jazzy/Harmonic;
this is not Humble Nav2 certification. Target CPU real-time performance, moving
obstacles, real hardware, calibrated tire slip and long-term drift remain outside
this acceptance. The conservative footprint critic has numerical collision tests;
obstacle navigation is not included in the physical tracking scenes.

Run `python tools/format.py --check` with the pinned Rolling ament tools. See CI
for exact standalone dependency installation and colcon commands. Version 1.0
removes the Python executor comparison because that executor no longer exists.
