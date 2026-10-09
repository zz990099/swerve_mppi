# Configuration

`Config` is a startup-only value object. Construct the controller and its models
from one validated configuration. `config/default.conf` contains the current full
set of defaults; unknown or duplicate keys, wrong numeric types and invalid values
are errors. There are no old configuration aliases.

- `config_schema()` lists names, units and numeric types.
- `with_config_parameters(base, overrides)` applies typed overrides transactionally.
- `parse_config_profile(text, base)` reads bounded locale-independent `name=value` text.
- `write_config_profile(config)` emits every current field at round-trip precision.
- `parse_resolved_config_profile(text)` requires every current field, for reproducibility.
- `validate_live_config(config, maximum_budget_ratio)` requires a positive bounded compute budget.

Profiles accept `#` comments and at most 64 KiB. Loading never modifies the supplied
base. An empty override profile preserves it. Resolved profiles do not inherit omitted
fields. These are planner configuration APIs, not a peer execution/arming handshake.

Geometry and wheel units describe the nominal model. Velocity, acceleration, steering,
transition and stopping parameters constrain prediction. Sampling, noise, critics,
navigation and planning-budget parameters control the optimizer and task policy.
`compute_budget_ratio=0` disables the wall-clock budget for offline deterministic
regressions; live callers must use a positive budget. Input size and stopping-horizon
limits bound work without truncating paths or obstacles.

Timing configuration has three independent periods: `planning_period_s` defines the
live compute budget, `model_period_s` defines the MPPI/trajectory grid, and
`chassis_period_s` defines command-mechanics substeps. Pairing, observation age/gap,
future tolerance, command lifetime and application uncertainty are explicit bounds.
History steering/wheel tolerances admit only reconciliation error; they do not relax
the strict encoder/body nominal model gate. All configured periods convert to positive
integer nanoseconds during validation.

Shared chassis defaults match the current Python configuration: body caps 0.8 m/s
and 0.8 rad/s, 100 Hz updates, wheel caps 2 m/s and 4 m/s^2, steering rate 2.5 rad/s,
0.05 rad measured alignment tolerance, 0.05 s alignment dwell and 5 s transition
limit. Steering travel is exactly +/- pi/2; unsupported older travel ranges are rejected.

`max_vx_mps`, `max_crab_speed_mps`, `max_spin_radps`, `max_yaw_rate_radps` and
`min_turn_radius_m` are planner proposal policies, possibly stricter than shared
chassis caps. `capture_linear_decel_mps2` and `capture_angular_decel_radps2` shape
terminal capture only. They do not simulate chassis zero/braking. Previous body
brake and affine-residual configuration keys have no aliases and are rejected.
Stage 4 will compare shared settings with the message adapter. Matching these defaults
does not establish physical Gazebo tracking or braking accuracy.
