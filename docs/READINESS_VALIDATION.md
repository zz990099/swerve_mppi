# 0.21 readiness validation

## Changes and contract

The extreme public trace `(0,0) -> (1e200,0)` with a circle at `(1e100,0)`
now fails closed instead of returning Valid. Shared geometry checks derived
products, normalized projections and turn angles. Public scoring also rejects
nonrepresentable geometry rather than propagating a geometry exception.

One immutable exact spatial frame is built inside each planning call's wall
budget and shared by its mode branches. It retains all ordered path segments,
all circle obstacles, curvature and earliest-segment tie semantics. Conservative
outward AABBs only prune impossible intersections; the narrow phase and all
custom hard constraints remain authoritative. The private prepared token admits
input once; every resulting trajectory still checks its anchor, poses and sweep
margins. Public and execution-time admission always validate fresh contexts.

## Verification

Safety tests include overflowing segment products, crossing/tangent circles and
curve-to-chord margins. A fixed-seed exhaustive long-double oracle checks exact
nearest queries on 1024 random segments. Prepared scoring matches exhaustive
public scoring on 2/401/4096-point paths with 128 circles, including changed
contexts and copied optimizers. Existing motion, complete stopping, transition,
feedback, installed-consumer and deadline regressions remain applicable.

## Live workload and remaining acceptance

`docs/benchmarks/readiness_v0.21_release.csv` and
`docs/benchmarks/readiness_v0.21_dense_release.csv` record 50 cold full-pipeline
calls per row on this shared x86_64 environment, using the live ROS ratio 0.6.
The dense probe uses 4096 points. On this shared host the final 128-circle
ordinary row recorded 2/50 timeouts and the dense rows 0/4/3 timeouts for 0/40/128
circles. The strict probes consequently returned failure despite much lower
median work than the audit baseline (128 ordinary circles: 50/50 timeouts).
These results must not be described as a passed target-host real-time gate. These are host-specific observations, not a
real-time guarantee for another CPU. Deadline exits truncate work and must be
read alongside timeout/overrun counts, never as complete solve measurements.
See PERFORMANCE.md for strict target-host commands.

The companion simulator adds source/receipt/context-age diagnostics, repeated
nominal physical probes and a 25% mass perturbation with unchanged control limits.
Physical model errors must satisfy its documented nominal envelope and per-mode /
braking truth-coverage gates. Slip/friction, servo-lag, independent localization,
Nav2 and hardware calibration remain outside this bounded acceptance.
