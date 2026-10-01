# Pre-simulation core corrections (0.7.0)

Baseline: 1c0d3124025991a23a247b7f1361843d6a8f0ed6 (0.6.0).
This work changes only the standalone C++ core, contracts and tests. No ROS,
Nav2, Gazebo, transport or hardware integration was added.

## Three batches

1. Path matching retains the current segment and anchors initialization to the
   first nonzero segment. Later segments require sequential endpoint passage;
   sharp corners/reversals require XY capture. Smooth sampling points can advance
   despite cross-track error. New regressions cover short loops, crossings,
   foldbacks, uncaptured corners, duplicates and dense samples, including corner
   capture updating the stored segment as well as arc progress.
2. Tracking, terminal and corner controls share control application and committed
   alignment. Policy changes cannot renew alignment or discard its intent before
   first Drive. Explicit switch rollouts preserve entry intent through first Drive
   and mask the ignored resume proposal. Immediate/future switches with different
   post-entry directions and capture steering faults have focused regressions.
3. A shared const TrajectoryValidator enforces swept circle collision and injected
   hard constraints independently of cost. Infeasible planning permits Brake/Hold
   only with a validated stopping trace that ends stopped. Unsafe stopping still
   latches SafeStop. Capture/alignment checks one Drive plus full stopping instead
   of constant extrapolation beyond a nearby goal. TimingGuard/TimedExecutor reject
   stale feedback, missed periods, old/missing/future commands, replay and wrong
   sessions. Verified stopped recovery renews session identity but preserves mode
   request high-water marks. Sparse/repeated goal observations restart settling.

## Validation

GNU 13.3.0, CMake 4.4.3, C++17, -Werror:

| Check | Result |
| --- | --- |
| Release, benchmark enabled | 18/18 CTest entries passed |
| Debug, benchmark enabled | All 18 entries passed, including two retried entries |
| AddressSanitizer + UndefinedBehaviorSanitizer | 7/7 core suites passed |
| Default behavior matrix | Nine scenarios, five seeds each, both Debug and Release |
| Installed consumer | Finds 0.7 and links controller, hard validator and timed executor |
| Allocation smoke | Existing peak-at-most-200 gate passed in Debug and Release |
| Formatting/whitespace | clang-format and git diff --check passed |

The first Debug run could not start two generated executables because their local
execute permission bits were absent. Restoring permissions and rerunning those
two entries passed; all other entries passed in the full run. Sanitizers use the
existing non-PIE configuration with detect_leaks=0, excluding behavior/installed
consumer/benchmark tests. No leak-detection claim is made.

Safety regressions verify temporary obstacle recovery without reset, unsafe
braking fault latching, measured capture before an obstacle, and identical injected
constraints for tracking/capture/alignment/standalone optimization. Timing tests
begin with accepted positive Drive before loss/replay checks, exercise a complete
guarded lateral handshake/capture loop, and verify old mode IDs remain rejected
after transport-session renewal.

Historical 0.6 timing CSVs remain unchanged. Version 0.7 does not claim identical
completion ticks or a new worst-case solve-time bound; its behavior acceptance
criteria and allocation gate were rerun.

## Integration contract

Rebuild downstream consumers with find_package(swerve_mppi 0.7 CONFIG REQUIRED).
New public diagnostics are NavigationStatus::Waiting, ControlPolicy::Blocked and
FailureReason::UnsafeStoppingTrajectory. Queued commands should use TimedExecutor
and CommandEnvelope; direct ModeExecutor remains the synchronous reference API.
See EXECUTION_CONTRACT.md for timing limits, clock domains and recovery.

The adapter must provide fresh consistent measured feedback, one clock domain,
serialized periodic calls, explicit mode feedback, session coordination and a
physical watchdog independent of process liveness. Braking/steering dynamics,
slip and feedback latency still need calibration against an independent plant.
Circular footprints, one-switch horizons, seeded entry directions and lack of a
solver deadline remain documented model limits, not completed simulation work.
