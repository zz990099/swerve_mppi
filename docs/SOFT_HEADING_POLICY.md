# Geometric body heading as a critic preference

Ordinary external paths specify ordered XY geometry and one final pose yaw.
Intermediate yaw is ignored. Body yaw is a predicted state chosen by MPPI;
ordinary tracking no longer rewrites the route into a prescribed effective-yaw
trajectory or enters a mandatory body-alignment maneuver.

The default critic set contains XY/execution tracking and a separate HeadingCritic.
The latter scores ordinary travel with

```
path_heading_weight / N * sum((1-goal_weight) * eta(s) * (1-cos(body_yaw-geometric_yaw))/2)
```

The complete travel contribution lies in `[0, path_heading_weight]` (default
maximum 0.5), independent of angle magnitude or sample count. Any physically
valid opposite heading can still compete. This bounded weighted sum expresses
preference, not a formal lexicographic guarantee of maximum progress.

For every cycle, the shared route profile precomputes

```
eta(s) = preview_length / heading_preference_distance
         / (1 + (accumulated_absolute_direction_change / heading_preference_turn)^2)
```

The preview is bounded by the supplied ordered local section. Defaults are 2 m
and 0.50 rad. Long straight preview produces eta near one; short preview and
repeated turns reduce it continuously. Weights depend on route geometry, not
candidate speed or candidate self-created curvature. No simple/complex mode
switch or extra maneuver state machine is introduced.

The common nominal wheel/speed envelope holds the current measured body heading
and uses XY direction changes. It imposes no geometric body-yaw derivative.
Every candidate's actual wheel/body evolution still passes through the original
conditioning, generator, rollout and hard critics. The terminal velocity cap is
retained because final yaw remains a task requirement.

Candidates include warm continuation, keep-body-yaw XY tracking, braking, slow
and short recovery, plus a gentle geometry-alignment proposal and exploration
around both proposals. Alignment guidance is optional, with 35% of the planning
angular-speed limit before actuator conditioning. Goal yaw has its own quadratic
cost and feedback; ordered measured terminal activation fades the travel preference.
Shared completion tolerance, validated zero output and precise internal tasks remain.

A physical wheel-alignment intent may be latched only from the optimizer's
accepted ordinary request. Exploration pauses while that accepted wheel target
is physically aligning; it is released on measured alignment or wheel motion.
This preserves execution continuity without imposing a geometric body-yaw target.
Explicit EffectivePath/internal maneuver contracts retain their previous semantics.

Diagnostics include `heading_preference`, bounded selected heading cost and
independent goal cost in the local recorder. ROS diagnostics expose the preference
and existing selected-cost breakdown. `travel_heading_scale` is removed; use
`path_heading_weight` for the maximum preference budget and the two preview
parameters for route context.

## Checks and limits

Core, policy contracts, support and relocated installed consumer pass; Python
recording/audit checks pass. Policy tests verify alternative candidates, finite
bounded opposite-yaw cost, contextual weighting, speed/yaw decoupling, independent
final yaw, hard rejection and retiming.

Historical strict closed_loop still fails its uniform geometric-heading metric
(short lateral case RMS about 1.565 rad); thresholds/statistics are retained for
visibility. stagnation case 0 (straight + final 180 degrees) still exceeds its
10 cm corridor at 16.15 simulated seconds. These are unresolved observations,
not claims of passing quality. The first pre-latch prototype stalled in crab_right;
physical selected-request latching was restored before the final frozen round.

This revision does not introduce an orientation-aware polygon collision model,
repair deep wheel-transition/terminal issues, or optimize realtime performance.
Full-round observations and retained regressions are recorded in SOFT_HEADING_VALIDATION.md.
