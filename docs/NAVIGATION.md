# Navigation and mode selection

Supply a nonempty ordered path with a stable `path_id`. Changing geometry or identity
restarts local path progress. Duplicate points, loops and reversals retain ordered
segment semantics. PathManager selects a local target with corner capture, curvature
speed limits and remaining distance; proximity to the final point alone cannot skip
uncaptured path segments.

FollowPath uses local path orientation; GoalOnly reserves final yaw for goal handling.
GoalManager separates tracking, approach, yaw alignment, stopped settling and completion.
Completion requires measured pose, speed, wheel and mode conditions for a dwell.
A completed task stays at a zero-velocity intent until replanning/reset.

ModeScheduler evaluates keep-mode and eligible switch branches. Switching cost,
hysteresis and minimum actual-mode age resist frequent mode changes. Private
ModeManager preserves request identity and entry intent while waiting for measured
acceptance and confirmation. A replan does not replace an already pending mode
request. Same-mode large steering changes retain a local alignment commitment.

The core predicts delay and motion during switching; it does not move the wheels or
synthesize real confirmation from predicted time. New constraints are checked during
pending alignment and normal stopping. No feasible trajectory yields a checked normal
stop when possible; an invalid/unsafe result withholds command authorization.
