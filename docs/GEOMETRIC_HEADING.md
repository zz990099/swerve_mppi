# Soft geometric body-heading preference

Intermediate path yaw remains ignored. Ordered path positions define a local
geometric direction; the final goal yaw remains an independent task objective.

`ReferenceProfile` caches a one-metre arc window at its speed-profile knots.
Sixteen uniformly spaced arc intervals estimate a chord direction, directional
coherence, absolute accumulated turning and signed average curvature. Opposite
turns do not cancel the complexity measure. Coherence and fourth-power turning
and angular-rate demand factors continuously reduce the preference. A degenerate
chord, a pure rotation or zero weight has no geometric-heading objective.

The tracking critic adds a normalized running Huber cost with a separate 0.35 rad
scale and weight 0.4. Misaligned candidates remain finite. Candidate speed and
candidate yaw error do not turn off the preference. In the final ordered segment,
the preference fades smoothly across one metre before the existing 0.08 m terminal
heading activation region. It never reads intermediate waypoint quaternion yaw.

`geometric_heading_allow_reverse` defaults to `false`, which prefers aligning the
body's forward axis with the ordered path. When enabled, the critic also accepts
alignment with the opposite tangent and gives forward a small tie-breaking
advantage around ±90°. Both choices advance along the ordered path. This is a
soft cost, so lateral motion and short reverse corrections remain feasible.

One additional structured candidate combines geometric yaw feedback and curvature
feedforward with the existing path-position feedback. It retains normal command
conditioning, actuator dynamics, collision checks and critic selection. Existing
free-heading, braking and recovery proposals remain. It is not injected during
latched wheel alignment, departure or terminal hold. Setting
`geometric_heading_weight` to zero also restores the previous proposal allocation.
On a simple path with a large yaw error, the candidate can turn before driving.
The critic temporarily relaxes the incentive to match cruise speed so that this
candidate can compete with immediate translation.

The default Nav2 goal tolerances are 0.05 m and 5 degrees. The ordered goal checker
supplies these tolerances to MPPI for terminal braking. The goal checker still
requires fresh, stopped chassis and wheel feedback and its configured settle time.

The geometric preference does not establish a strict task hierarchy. Position,
progress, effort and actuator costs can outweigh it. The existing reference-speed
envelope remains conservative and assumes a fixed body frame; this change does
not redesign speed planning or wheel-branch maneuvers. Collision checks still use
a conservative swept circumscribed disk, so they do not model the passage benefit
of rotating a rectangular body in a tight obstacle corridor.

Validation includes geometry/density/yaw invariance, cusps and opposite turns,
ordered crossings, soft rejection semantics, terminal fade and parameter checks.
Physical PR jobs retain the existing 32-case gates and add a continuous 100 m
route, seed 42, maximum 1 m/s, map path and odom control. Partial recordings are
retained when a case fails; failures do not relax any geometric or timing gate.
