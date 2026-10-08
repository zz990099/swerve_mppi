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

The current model defaults are not a validated match for Python/Gazebo execution.
Stage 2 reconciles the model, and stage 4 compares shared chassis settings without
requiring the chassis to understand optimizer parameters.
