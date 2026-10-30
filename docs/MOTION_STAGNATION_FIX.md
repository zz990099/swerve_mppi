# Ordered motion and steering branch recovery

The 2026-10-29 baseline (`0ad53ea`, physical run 40850050500) completed 29 of
32 routes. `heading_sweep`, `spiral_out`, and `stop_turn_go` stopped until the
Nav2 PoseProgressChecker aborted them. This change addresses motion progress
and request slew; it does not optimize computation time or relax quality gates.

## Route semantics

`swerve_mppi::OrderedPath` retains coincident-position heading changes as
rotation sections. Its progress consists of a section index and translation
arc within that section. A translation preceding a rotation exposes its own
endpoint as the tracking goal. A rotation exposes its target heading and blocks
later sections until measured pose and stopped velocity meet the transition
thresholds. Global goal readiness is independent of this intermediate tracking
goal. The pose-only overload assumes stopped velocity; execution callers must
supply measured velocity.

The Nav2 adapter transforms the current section's tracking goal into the local
costmap frame. It clears optimizer warm state when the section changes, while
retaining the previous published request for slew conditioning. Rotation-only
candidates slew translation to zero during the turn, then may correct residual
position error as heading settles.

## Steering branches

A greater-than-90-degree change in the bounded steering representation is a
stop/reorientation event, not a permanent zero-speed interval in the spatial
reference. The shared actuator generator still enforces braking, steering rate,
wheel acceleration and rolling feasibility.

At measured standstill, the controller retains a steering intent until measured
alignment or resumed driving. Near an identified reference branch boundary it
uses the outgoing branch. Candidates start from this deterministic intent while
alignment is pending, and keep the validated brake candidate. The initial
intent is released inside a rollout once predicted driving begins, so it cannot
be reused at a different future event.

The alignment objective compares the wheel rolling axis modulo pi during the
initial measured reorientation; actuator cost retains the actual bounded-angle
travel. A short departure-then-brake candidate provides a validated alternative
to a full-horizon commitment through another event. The alignment objective
charges the initial measured reorientation. Later stops
in the same horizon retain physical actuator cost and reference-time pausing,
but do not accumulate the initial-event alignment penalty for every remaining
prediction step. This prevents short-horizon parking from avoiding an unrelated
future event's entire alignment penalty.

## Publication timing

Nav2 Jazzy replaces a plugin command's timestamp after its compute callback.
Thus differences between compute-start times do not necessarily equal the
intervals in the published stamped command stream. Before returning a result,
the adapter checks the available completion interval. If it is shorter, the
controller conditions the selected sequence again, rolls it out, and re-runs all
critics. Only a validated adjusted sequence becomes the outgoing request and
warm start. A 1 ms handoff allowance is reserved. Exceptions and fault stops
retain Nav2's existing safety behavior.

## Diagnostics and regression

Recordings retain candidate totals and tracking components for mean, fresh,
brake, slow, recovery and selected sequences. Components are path, heading, velocity,
alignment, effort, progress and goal. Totals include injected costmap/custom
critics. Diagnostics also expose route section, rotation state, remaining arc,
latched reorientation, reference branch distance, noise scaling and reference
limiting flags (geometry/end 1, yaw 2, wheel speed 4, corner 8, actuator rate 16, acceleration envelope 64).
The chassis execution stream remains the authority for actual presteering.

Portable tests cover rotation ordering, wraparound, measured stopping, shortened
publication intervals and rejection during retiming. `stagnation` runs the three
formerly stalled 12 m routes and the self-crossing cloverleaf route with the shared executor and an independent servo
plant. The cloverleaf functional regression permits up to 90 simulated seconds; this
does not change the physical wall-time gate. The physical workflow remains one 32-case round with seed 42. Its 50 ms
P99 gate is unchanged, so real-time failures remain visible until separately
addressed.

## Explicit transition and ordered rejoin

A stopped steering branch change keeps a short maneuver until measured body
alignment, wheel alignment and departure complete. The departure retains its
translation/yaw coupling and rejoins while rolling when heading is within 0.03 rad;
a larger measured error requires stopped heading settling. Its chord
is bounded by the source path corridor and cannot pass a mandatory corner.
Wheel alignment uses a small twist below the stopped-wheel threshold; it still
passes through the same actuator rollout and collision checks. Returning to the
original route uses a bounded ordered rejoin and retains rotation barriers.
The effective reference is retained for publication retiming, so retiming
validates the maneuver actually being executed.

A self-crossing local preview must not initialize the optimizer at a later loop
that happens to be closer to the measured pose. Initial matching is limited to
the adjacent 0.3 m preview and respects the next mandatory corner. During
recovery, the retained steering intent includes the same cross-track and heading
correction as the feedback seed; it is not just the path tangent.

Request slew is evaluated from the published command stream and its command
stamps. Chassis-reported accepted-request changes are retained separately as
`max_feedback_linear_rate` and `max_feedback_angular_rate`; watchdog or feedback
state changes must not be attributed to a nonexistent publication. Thresholds
remain unchanged.

The nominal steering filter also checks the effect of feedback correction on
wheel rolling axes. A fixed 2 cm pose threshold alone can suppress a large
steering correction at low reference speed and make alignment and tracking
alternate indefinitely. Corrections exceeding half the tracking-angle tolerance
retain their feedback steering intent throughout alignment and drive startup.
Route progress searches a nearby forward arc using measured displacement as a
budget, so a missed sampled tangent on a smooth curve cannot freeze progress.
Mandatory corners and rotation sections remain barriers.

Feature branches are validated by pull-request workflows; pushes to `main`
remain validated directly. This prevents running the same physical round twice
for a feature-branch push and its pull-request synchronize event.

At a tightly settled terminal pose, the controller validates a slew-limited
zero-request hold without exploratory noise. Ordered goal readiness additionally
requires a zero outgoing request, so Nav2's final zero publication cannot truncate
a still nonzero requested yaw rate while measured wheels happen to be stopped.
