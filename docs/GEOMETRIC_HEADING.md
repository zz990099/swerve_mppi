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

One additional structured candidate combines geometric yaw feedback and curvature
feedforward with the existing path-position feedback. It retains normal command
conditioning, actuator dynamics, collision checks and critic selection. Existing
free-heading, braking and recovery proposals remain. It is not injected during
latched wheel alignment, departure or terminal hold. Setting
`geometric_heading_weight` to zero also restores the previous proposal allocation.

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
