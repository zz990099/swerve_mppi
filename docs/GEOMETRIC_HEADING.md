# XY path following with independent final orientation

The current implementation treats intermediate body heading as a bounded,
route-dependent critic preference. XY route and final pose yaw remain task inputs.
See [current policy](SOFT_HEADING_POLICY.md) for scoring, candidates, physical
constraints and terminal behavior.

The previous effective-yaw profile approach and its results remain documented in
[local optimization history](GEOMETRIC_HEADING_LOCAL_OPTIMIZATION.md) and the
historical Gazebo analysis. They do not describe or validate the current revision.

Use [local testing](LOCAL_TESTING.md) for incremental single-case and full-round
recordings. Position, terminal and physical safety gates are unchanged; a soft
travel-heading preference is not an absolute tangent-heading acceptance condition.
