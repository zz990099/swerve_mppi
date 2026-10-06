# Shared configuration contract (0.21.1)

`common/config_profile.hpp` provides startup-only configuration metadata,
transactional overrides, portable profiles and planning/execution compatibility
checks. The core remains C++17 and ROS-independent. `Config` layout, defaults,
controller algorithms and actuator semantics are unchanged.

## One complete registry

`config_schema()` describes all 68 current `Config` fields by name, unit, numeric
type and scope. `config_parameters()` extracts validated values. Integer fields
use `uint64_t` at the boundary and reject narrowing into their actual field type;
real fields use finite `double`. `with_config_parameters()` rejects unknown names,
duplicates and wrong variant types. It validates the completed configuration,
allowing coupled changes without depending on override order. The base is never
modified, including on failure.

`config/default.conf` records every default with units and scope. CMake installs
it in `${CMAKE_INSTALL_DATADIR}/swerve_mppi`. These are nominal research defaults,
not measured plant parameters. Keep the registry, example and regression coverage
in sync when adding a `Config` field.

## Profiles

Profiles use one `name = value` assignment per line; blank lines, CRLF and `#`
comments are accepted. Reals support decimal exponents, independent of global
locale. Integers require unsigned decimal notation, without signs, fractions or
exponents. Unknown and repeated names, trailing junk, nonfinite numbers, overflow
and values too small to represent are rejected. Profiles are limited to 64 KiB.
An empty profile preserves a validated base. Syntax/type errors report the line
and parameter; final cross-field validation reports the existing core diagnostic.

`write_config_profile()` emits all values with enough precision for exact reload.
The format is a portable offline artifact, not ROS YAML. A future ROS adapter can
translate typed parameters through `with_config_parameters()` and publish the
resolved values without placing ROS dependencies in the core.

```cpp
#include "swerve_mppi/common/config_profile.hpp"

auto planner = swerve_mppi::parse_config_profile(
  "max_linear_accel_mps2 = .5\ncompute_budget_ratio = .6\n");
auto executor = planner;
swerve_mppi::validate_live_config(planner, .6);
swerve_mppi::require_execution_compatible(planner, executor);
// Construct Controller/TimedExecutor/ProfileRunner from these checked values.
```

These APIs allocate and must run during setup, never in actuator callbacks.
Existing constructors retain their current `validate(Config)` behavior; the new
live-budget and compatibility checks require explicit caller use.

## Shared execution settings

The 33 `Execution` settings include chassis geometry and footprint; wheel/body
limits; steering, stopped and feedback tolerances; mode dwell, minimum alignment
and confirmation timeout; model interval; context input limits; and stopping
horizon. They must match exactly across planning and execution because prediction,
admission, mode protocol and stopping certification depend on them.

The remaining `Planning` settings include optimization horizon, sample count,
noise, costs, prediction-only confirmation allowance, switch preferences,
continuation reduction attempts, navigation capture/progress settings and compute
budget. They may differ without an execution compatibility failure. This scope
describes agreement requirements, not whether a setting can be tuned live.

`execution_config_mismatches()` validates both configurations and returns all
differing shared names in registry order. `require_execution_compatible()` rejects
the first difference. Comparison is exact, including representable differences
below common tolerances; a digest or approximate comparison is not used as proof.
Both configurations must be available to the caller. This API does not discover
another process's configuration or enforce startup agreement over a transport.

`validate_live_config()` rejects zero compute budgets and unrepresentable budget
durations, optionally imposing a tighter ratio for transport headroom. Zero
remains supported by ordinary core validation for deterministic offline tests.

## Reproducible pipeline measurement

```sh
./build/swerve_mppi_integration_budget 50 --config config/default.conf \
  --budget-ratio .6 --write-config budget.conf > budget.csv
./build/swerve_mppi_integration_budget 50 --config budget.conf \
  --path-points 4096 > budget-dense.csv
```

Precedence is core defaults, then profile overrides, then an explicit CLI budget
ratio, independent of argument order. The resolved profile records actual values
used by all three pipeline consumers. The tool rejects profiles whose input
limits cannot admit its full workload rather than shrinking the workload. Keep
the profile, CSV, source SHA and build/host details together. Profiles with other
valid model parameters may cause functional workload rejection; accepting a
profile does not certify tracking for that configuration.

The companion ROS and Gazebo packages have not been migrated to this API in this
stage. Their existing parameter mapping and nominal closed-loop evidence remain
separate. See [PREPARATION_PLAN.md](PREPARATION_PLAN.md) for the remaining work.
