# Queued entry and full-interval residual fixes (0.19.1)

Review baseline: 68d5d297cc724093fda28139e56ee877a3685b97 (0.19.0).
Scope: portable planning/execution contracts; ROS/Gazebo physical validation remains
integration work.

## Queued mode-entry geometry

A request planned with +0.01 rad steering can choose +pi/2 for lateral Crab entry.
If steering is -0.01 rad when the queued body request is first accepted, execution
can choose -pi/2 and reverse the eventual wheel signs. Both represent the same
body intent within mechanical stops. Previously ModeManager compared the earlier
exact planning angles and eventually timed out after healthy execution confirmation.

ModeManager now compares rolling lines modulo pi. Numeric/mechanical input admission,
matching actual mode, confirmation, request ID and stopped body/wheels remain
required. The executor retains its exact immutable mechanical request and deadline;
this change only affects measured acknowledgement. Handover preserves measured
angles instead of steering back to the prediction. No wrapped mechanical shortcut
is commanded, and the public request still contains only frozen body intent.

Chassis regressions run both signed lateral goals with a 0.1 s queued request and
the opposite execution-start steering. The independent profile fixture consumes
checked ProfileRunner samples through mode confirmation, first Drive and completion.
Additional cases cover equivalent representations for all three target modes,
wrong IDs, wrong actual modes, absent confirmation, wrong rolling lines and
moving wheel feedback.

## Complete Drive residual envelope

The reviewed straight-to-curved command (0.4 m/s, yaw target 0.1 rad/s) has ideal
joint endpoints but an approximately 0.00048041 m/s midpoint module residual.
Setting drive_kinematic_tolerance_mps to zero previously still admitted this profile.

The shared certifier evaluates residual against instantaneous encoder FK throughout
the affine joint interval, including its measured start. For wheel speed change ds
and steering change da over tick fraction f, a module rolling vector has second
derivative bounded by B_i = hypot(2*ds*da, max_abs_speed*da^2). The mean bound is
B = sum(B_i)/4. All four rectangular chassis modules have equal radius, so removing
body translation and rotation gives residual second derivative bound B_i + 2*B.
An interval of width h is enclosed by its residual endpoint chord plus
(B_i + 2*B)*h^2/8. Vector-norm convexity then bounds the whole interval.

Fixed steering gives affine residual vectors; endpoint bounds suffice in that
case. Identical wheel speed/steering pairs throughout a common translation give
zero residual, including moving steering. Other intervals use adaptive enclosures
with a 4096-node budget and maximum depth 14. Samples can reject observed excess;
only analytic enclosures authorize intervals. Exhaustion fails closed.

DriveModel reduces the joint step when certification fails. ModeExecutor and
ActuationModel use the same interval check before returning healthy Drive targets
or a profile. The tolerance includes the existing 1e-9 m/s numerical allowance.
Zero tolerance can sharply reduce or reject moving-steering progress; use a
nonzero envelope calibrated against the chassis when that motion is intended.
An incompatible measured start cannot authorize Drive merely because its endpoint
is ideal. The independently validated proportional Brake path remains available.

Actuation regressions use independent encoder equations to expose the original
interior excess, reject it at zero/tight tolerances and accept an adequately sized
envelope. Dense profile checks cover model-reduced commands and the public
TimedExecutor body-command pipeline. Exact common translating steering remains
admissible; an incompatible measured start is rejected for Drive and retains Brake.
An additional curve has ideal endpoints and a compliant midpoint but an excessive
peak near f=0.5026; it is rejected at 0.007821828 m/s and accepted at 0.008 m/s.
This guards against replacing the analytic enclosure with endpoint/midpoint sampling.

## Verification

Local validation used GCC 13.3, C++17 and -Werror:

- Release: all 56 CTests passed (101.22 s).
- Debug: all 56 CTests passed (366.88 s).
- Both runs include the queued-entry and full-interval residual regressions,
  independent profile execution, installed consumer and 19 standalone public
  header compilation probes, plus the pinned ROS Rolling format check.
- A final official-format check and git diff --check also passed.

The Release production integration-budget probe ran separately after both test
runs, using the default 80 ms budget and 50 repetitions per obstacle count:

| Obstacles | p50 (ms) | p95 (ms) | Maximum (ms) | Timeouts | Total overruns | Other failures |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 33.7244 | 36.3910 | 37.1549 | 0 | 0 | 0 |
| 40 | 43.1967 | 45.0963 | 56.8579 | 0 | 0 | 0 |
| 128 | 65.1899 | 70.3133 | 78.6641 | 0 | 0 | 0 |

The 128-obstacle maximum leaves approximately 1.34 ms of budget headroom on this
host. These measurements are evidence for this configuration and host, not a
real-time guarantee. Integration must measure the target machine and select its
workload/budget accordingly; deadline failure remains fail-closed.
