# Companion audit and offline migration preparation (0.21.5)

The core and companion have useful boundaries; retain the planner, typed chassis
protocol, execution supervisor and sole joint writer. No wholesale rewrite is
needed. This batch audits those boundaries and implements offline preparation
only. It does not change the companion, update its runtime core pin, launch ROS or
Gazebo, or certify physical behavior. The previous four core preparation stages
remain complete.

## Reproducible baseline

The reviewed companion is
[`zz990099/swerve_gazebo_sim@d05e0a0`](https://github.com/zz990099/swerve_gazebo_sim/tree/d05e0a037dec986c167fd58f8bd2fb1ff56b5723).
Its CI pins core
[`275983c`](https://github.com/zz990099/swerve_mppi/tree/275983c9906dc526e67050a34eddb7eb37cfb448)
(0.21.0); `find_package` also requests at least 0.21.0. Neither automatically
incorporates the contracts added in 0.21.1..0.21.4.
`preparation/simulation/baseline.json` records these revisions and the Git blob
hashes of 38 audited files. This proves the reviewed file set, not the provenance
of every file in an arbitrary directory. Drift or omission rejects preparation;
repeat the audit and review the baseline update before using a newer companion.

The old and new Config declarations have the same numeric defaults. The current
explicit ROS nodes each expose only nine shared core overrides. The planner also
declares a compute-budget ratio, default 0.6. Our full candidate profiles are
reconstructed using an explicit complete base and those overrides. They are
**not configuration exports from running participants**. Live export and startup
exchange remain migration requirements.

## Existing ownership and gaps

References in this table are companion-relative paths at the pinned revision.
The tool does not infer runtime correctness from source spelling or passing
startup comparison. Its remaining gates come from this reviewed audit.

| Boundary | Existing source behavior | Required migration or proof |
| --- | --- | --- |
| Configuration | `src/mppi_planner.cpp`, `src/chassis_controller.cpp`, `launch/mppi.launch.py`, `swerve_gazebo_sim/bringup.py`: nine duplicated shared overrides; other fields inherit core defaults | One complete resolved source per participant; validate live budget; exchange all fields and compare execution scope before arming. Include frame/clock/policy/timing metadata. |
| Explicit owner | `bringup.controller_config` selects one ChassisController and removes forward/guarded controllers in chassis mode; its runtime embeds EndpointGuard | Keep one final joint writer. Do not insert cmd_vel inference or a second competing controller. External-joint mode uses a different GuardedJointController and is not the chosen migration path. |
| Clock | PhysicsClock captures the authoritative Gazebo PreUpdate time; StampedGazeboSystem exports the time with the synchronous joint read | Preserve that authority and original integer nanoseconds. `PhysicsTime` already stores double seconds, and no clock domain/reset epoch travels on the messages. Casting that double back to ns cannot recover the original stamp. |
| Snapshot | ChassisRuntime derives twist and integrates odometry from the measured joints; PlannerBridge retains timestamped context history, accepting bounded older context | Bind raw stamp/frame/clock metadata to actual payloads and the independent current application read. Preserve obstacle/context age rather than refreshing old observations. Explicitly define current-context assembly and uncertainty. |
| Independent evidence | `controller.py` and ChassisRuntime both use encoder FK. Truth is available only to test measurement through optional Gazebo Transport, with a separate world frame and 50 Hz rate | Encoder agreement cannot prove body motion or stopping. Truth is neither a runtime observation input nor automatically coherent with 100 Hz joint reads. Design alignment/provenance and bounds before independent acceptance. |
| Command | ChassisCommand already has session, sequence, source/issue/application/expiry stamps, explicit mode request and the complete bounded source path; decoding preserves source task | Keep these fields and all existing lifetime/task checks. Add clock domain/epoch and schema binding before converting time to seconds. Fresh issue time never refreshes source time. |
| Mode receipt | ChassisState and planner conversion preserve request ID, accepted mode, entry velocity and committed steering | Keep the complete executor-owned receipt immutable and echo it unchanged. Do not regenerate entry geometry in the planner. |
| Fault and cancellation | Planner publishes absent authorization; runtime stop sets fault, removes pending/prediction state and deactivates its guard; fault gates prevent further Drive | Add ProfileRunner::cancel to revoke the installed profile, serialize it with the final write, and discard queued/copied samples. Existing fault gating is useful and must remain. A valid later packet must not clear the latch. |
| Recovery | Arm checks a strictly newer session, zero command, matching task, encoder-derived stopped state and mode/steering rules; planner requires 0.1 s context streaming | Context streaming is not continuous independent stop evidence. Bind physical stopped acknowledgement and drain to the endpoint/session/recovery attempt; require fresh sustained body and wheel stop and coordinated explicit reset. |
| Endpoint watchdog | EndpointGuard checks expiry and steady time on controller updates; faults command zero wheel speed and hold measured steering | It is independent of planner publication, but runs in the actuator update thread/process. It cannot enforce a deadline while that thread is blocked or the process is dead. Measure update progress and define an independent stop mechanism for those cases. |
| Physical and CPU acceptance | Existing tests measure relative installed-profile prediction against independent truth, coverage, braking/modes and scheduling diagnostics; CI covers two ROS/Gazebo families | Historical nominal results belong to the pinned runtime. They do not prove independent admission, robust braking/slip envelopes, epoch resets or the intended CPU's deadlines for the candidate. |

The two endpoint alternatives are often confused: external-joint mode exposes the
14-double v1 packet to GuardedJointController; explicit chassis mode owns joints
directly and feeds the same guard internally. That packet restricts IDs below
2^53 and has no epoch. ChassisCommand's uint64 IDs do not remove the embedded
endpoint's current bound. Preserve existing rejection until a reviewed protocol
change establishes a different bound.

## Configuration map and deliberate assumptions

The offline runner evaluates only the exact verified pure `bringup.py` bytes. It
reproduces `mppi.launch.py`'s chassis-mode selection in memory, obtains the real
controller configuration from `controller_config`, and mirrors the pinned planner
mapping. No launch module or ROS package is imported.

| Core field | Companion source and conversion | Bundled value |
| --- | --- | --- |
| wheelbase_m / track_m / wheel_radius_m | geometry.wheelbase / track_width / wheel_radius | 0.6 / 0.5 / 0.1 m |
| max_wheel_speed_mps | control.max_wheel_speed [rad/s] × radius [m] | 2 m/s |
| max_wheel_accel_mps2 | control.max_wheel_acceleration [rad/s²] × radius [m] | 4 m/s² |
| max_steer_rate_radps | control.max_steering_rate | 2.5 rad/s |
| confirmation_timeout_s | control.mode_switch_timeout | 5 s, overriding the core's 2 s |
| robot_radius_m / collision_margin_m | actual safety_parameters geometry enclosure / safety margin | 0.5 / 0.05 m |
| planner compute_budget_ratio | planner node's declared default | 0.6, a 60 ms compute budget for dt=0.1 s |
| Remaining fields | supplied complete base.conf | Explicit core assumptions; no calibration claim |

Do not copy similarly named Python cmd_vel settings into the explicit supervisor.
Python `mode_dwell_time=0.1` means stable requested-mode duration; core
`minimum_mode_dwell_s=1` means age of the confirmed actual mode. Python alignment
duration is 0.05 s; the explicit core keeps 0.25 s. Python stopped-wheel threshold
is 0.05 rad/s; the explicit core keeps 0.005 m/s (numerically equal after conversion
only for the bundled radius). Steering travel is also inherited by the explicit
core rather than read by its nine-field map. The YAML requires pi/2 today, which
matches the inherited value; future changes must include this shared setting.

Body braking/yaw bounds, driving kinematic tolerances and independent residual
limits remain model assumptions. The 4 m/s² wheel-command ramp is not a measured
body braking guarantee. NominalEncoderOnly is selected for preparation and future
development tests only. Switching metadata to IndependentNominal does not supply
independent observations or make noisy/slipping feedback admissible. Keep strict
nominal admission; robust uncertainty/stopping work is a separate requirement.

The current explicit body's frame is `<prefix>base_footprint`; YAML's odometry
child override affects Python odometry only. The runner reports an additional
gate if they differ, or if a supplied core base has different steering travel. Namespace/prefix resolution uses the real companion helper.
No TF conversion is performed. The proposed clock domain and epoch in the bundle
are explicit offline placeholders, not live epoch allocation or reset handling.
Profile watchdog=0.15 s and TimingLimits (.15, .15, .25) describe the candidate
core owner; endpoint wall timeout=.1 s and per-sample validity<=.03 s are separate
guard settings and must be retained and measured independently.

## Offline preparation command

Requires CMake 3.20+, C++17, Python 3.9+ and PyYAML 6.0.3 (already pinned by our
developer requirements). Use an existing checkout of the companion at the audit
revision. CMake remains the only C++ build entry point.

```bash
cmake -S . -B build-preparation -DCMAKE_BUILD_TYPE=Release \
  -DSWERVE_MPPI_BUILD_PREPARATION_TOOLS=ON
cmake --build build-preparation --parallel 2
python tools/prepare_simulation.py \
  --tool build-preparation/swerve_mppi_config_preflight \
  --companion ../swerve_gazebo_sim \
  --output-dir build-preparation/simulation-candidate
```

Optional `--config PATH` supplies a custom startup YAML candidate; audited source
still must match. `--namespace`, `--prefix`, `--clock-domain` and `--clock-epoch`
are explicit proposed identities. Duplicate YAML keys are rejected rather than
silently overwritten. Output must be a fresh directory outside the companion;
previous evidence is never overwritten.

Artifacts contain complete planner/executor profiles, partial overrides used to
construct them, separate complete metadata, all 136 participant/field provenance
rows, copied YAML/base/baseline inputs, preflight log, hashes and a report. The
manifest records tool version/binary digest, preparation-source and CMake-cache
digests, the observed local Git HEAD/dirty state when available, audited revisions
and all artifact
digests. Status is `candidate_prepared_integration_blocked`; `runtime_export`,
`integration_ready` and `physical_acceptance` remain false. Exit 0 means successful
preparation, **not permission to connect or arm**. Failed attempts retain an
`incomplete` manifest and available diagnostics; they do not produce a readiness
claim. Artifact hashes detect drift, not authentication or physical correctness.

The C++ preflight also supports independent candidate editing and comparison:

```bash
build-preparation/swerve_mppi_config_preflight schema
build-preparation/swerve_mppi_config_preflight resolve BASE.conf OVERRIDES.conf
build-preparation/swerve_mppi_config_preflight compare \
  PLANNER.conf PLANNER.metadata EXECUTOR.conf EXECUTOR.metadata
```

Resolve requires a complete base, then uses the core override parser and live
validation. Compare requires two complete profiles and two complete metadata
records and calls AdapterContract/require_adapter_compatible. Planning differences
may pass; any execution or identity/timing difference fails. Numeric bounds,
duplicate/unknown/omitted fields and input >64 KiB fail. The preflight is an opt-in
build-tree tool, not installed runtime middleware or a substitute for live startup
exchange.

## Next preparation batches and later integration

1. **Next, still offline:** specify versioned wire records and implement bounded
   decoding fixtures preserving original integer ns, clock epoch, command/task
   identity and accepted receipts. Test reset/restart, epoch/session separation,
   stale whole snapshots, malformed frames/units and replay. Define the same-read
   integer physics-time side channel; do not fabricate original ns from double.
2. **Then, still offline:** specify the serialized adapter owner and recovery
   acknowledgement protocol. Exercise profile revocation versus copied samples,
   continuous independent stopped dwell, queue drainage and endpoint/session-bound
   acknowledgements. Keep endpoint progress/process-loss obligations explicit.
3. **Then prepare physical measurement:** define independent truth alignment,
   coverage and calibration datasets, braking/steering lag and uncertainty
   acceptance. Nonzero error bounds cannot bypass the nominal gate; accepting
   them requires robust swept/stopping/transition bounds first.
4. **Only after a separate integration instruction:** migrate the existing ROS
   nodes and messages, update the core pin, then run stepped nominal closed loops,
   directed transitions and injected fault/recovery tests. Measure target CPU
   deadlines under realistic contention. A Nav2 plugin can follow later if needed.

Validation for this batch is portable core/installed-consumer tests, offline CLI
failure tests and the source-pinned preparation regression. CI checks out the
companion only to read these sources and build candidate artifacts. It does not
run the companion's ROS/Gazebo workflow. Runtime and public core library behavior
remain unchanged.

The 0.21.5 preparation regression covers 51 cases, including each of the 38 audited
files drifting, a missing source, changed radius/geometry and unit conversions,
namespace/frame mapping, mechanical travel disagreement, duplicate YAML, incomplete
base, invalid identity and preserving failed/prior evidence. The standalone CLI
covers 54 cases, including independently valid peer mismatches, every omitted
metadata field, integer overflow/narrowing, nonfinite values, incomplete profiles
and bounded input. Both use the core contracts; no ROS-generated types are needed.
Debug UBSan additionally checks adapter/configuration/preflight regressions.
