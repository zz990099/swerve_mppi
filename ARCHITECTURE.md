# Continuous controller architecture

## State and prediction

Core input contains ordered local path, terminal goal, current pose, measured
steering angles and wheel speeds (rad/s), and the complete shared generator state
(previous target steering/speed arrays and valid flag), plus the last accepted
requested velocity. Pose/velocity measurements
and generator targets are deliberately distinct. A missing/invalid snapshot does
not silently become an ideal holonomic chassis.

Physical geometry and limits live in `Config::geometry` / `Config::limits` using
the shared core types. Planner speed caps may only tighten physical caps. ROS
checks every physical field against the chassis status before permitting motion.

Every prediction substep calls the same `Generator::step` as the executor, then
advances nominal actuator response and exact constant-twist SE(2) integration.
Steering uses a first-order servo with configurable `steering_time_constant`
(default 0.05 s), followed by the shared rate limits. This models the finite
position-servo settling time; targets and measured steering remain distinct.
The time constant is a prediction parameter to calibrate for the actual servo,
not an extra physical limit or an execution mode.
Predicted measured wheel inconsistency is bounded by `max_residual` (RMS m/s);
invalid trajectories cannot become valid merely because a later critic writes a
finite score. The forward wheel solution is an estimate, not a tire/slip model.
Pose is integrated from the prior measured nominal wheel state before advancing
response. Substep actuator effort and residual costs cover the entire horizon.

## Sampling and costs

One continuous three-dimensional sample batch replaces mode branches. Structured
candidates include warm start, arc-length path guidance, zero request and slower
guidance. Temporally correlated noise explores vx/vy/wz together. Softmax updates
are rolled out and rescored; a feasible weighted update within 0.02 normalized
cost of the best candidate is preferred, with the best candidate as fallback.
Noise attenuates continuously with wheel alignment; effective sample count is
reported. Requests obey vector/angular slew from the last accepted request. Near
steering representation boundaries, twist half-space projection preserves the
current rolling branch; unavoidable changes brake first. This never extends the
physical +/-90 degree steering range. Warm start advances with elapsed model intervals and is reanchored to
fresh measured/target state on every compute. Invalid state, new tasks and clock
resets clear it; ordinary presteering does not.

Path yaw is the desired body orientation, not inferred from the path tangent.
Lateral tracking can keep yaw constant. Body-yaw feedforward follows the path
reference, while curvature/corner preview reduces speed before actuator demands
become infeasible. Ordered progress is geometric arc length, with no waypoint
acceptance-radius pruning of a dense curve. Terminal activation uses remaining
arc length, and the local path retains later turns for preview. Critics cover
normalized path errors, ordered progress/remaining distance, command effort,
cross-cycle request smoothness, normalized steering/drive changes and consistency.
Predicted path/heading corridor violations reject a candidate; direct Euclidean
terminal attraction is not allowed to buy shortcuts through a curve. NoSolution returns
a zero request; it does not guarantee instantaneous stopping of a moving vehicle.

The grid critic conservatively sweeps a circumscribed footprint disk along each
predicted segment, sampling at half-cell spacing and checking touched grid cells.
This protects rotation as well as lateral motion but can reject narrow passages
that an oriented polygon could traverse. It is not a dynamic obstacle predictor.
Physical obstacle navigation requires its own scene-level acceptance.

## ROS boundary

Nav2 owns action lifecycle, command frequency, cancellation and final speed
publication. The controller only returns TwistStamped to Nav2; it has no direct
wheel publisher or command-context side channel. The default Jazzy server emits
stamped cmd_vel directly to the continuous chassis. Any downstream smoothing must
preserve vy and be separately included in prediction/validation.

The plugin checks both state publication and encoder source times, steady receive
silence, geometry/limits, fault flags and generator validity. A healthy fresh
faulted chassis receives a zero request via Nav2 for recovery; motion is retried
only after a valid snapshot arrives. Missing feedback fails control instead of
inventing an execution state. Replayed publication stamps do not refresh receive
silence; simulation clock rollback clears old snapshot and warm start.

Ordered path progress still prevents early completion on closed/out-and-back
paths. OrderedGoalChecker additionally subscribes to chassis health/stopped
feedback, so stale data or a fault cannot be hidden by a previous ready flag.
Core contains no task server or task completion output.
