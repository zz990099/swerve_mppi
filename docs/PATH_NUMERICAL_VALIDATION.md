# Derived path geometry (0.20.1)

Review baseline: 8976f6382b13fdfee76bae0dc75ab48d3852968f (0.20.0).

With a vehicle at `(1e308, 0)` and path `(-1e308, 0) -> (0, 0)`, all
input coordinates and the total path length are finite. Path projection and
cross-track subtraction nevertheless overflowed, and Controller emitted a
recoverable `NoFeasiblePlan` zero target with infinite cross-track diagnostics.

PathManager now rejects nonfinite derived lengths, projection ratios/bounds,
observed displacement, interpolation, cross-track errors and local-target distance.
Projection uses normalized segment components rather than squared length;
turn angles use normalized vectors rather than unscaled dot/cross products.
This preserves representable large-segment projections and blocking corners.
No arbitrary absolute coordinate limit is introduced. Unrepresentable derived
geometry fails closed before clamps or comparisons can mask it; projection applies
directional cancellation before rescaling an overflowing offset-to-length ratio.

Derived-error rejection clears the partial path matching cache and throws
`invalid_argument`. Controller's existing error boundary maps it to `InvalidPath`,
absent command authorization and finite default navigation diagnostics. The
actuator cancellation/recovery contract remains unchanged. A repaired path is
matched afresh; ordinary finite path tracking retains its ordered-window policy.

Regressions cover both coordinate axes and signs of subtraction overflow,
overflowing projection ratios and Euclidean errors, representable `1e160` signed
segments, a large blocking corner, local-target distance overflow after a finite
nearest error, a remote perpendicular observation with finite projection despite
an overflowing offset-to-length ratio, and cached-displacement rejection followed
by a valid observation.
The public Controller checks authorization, fault classification and finite
diagnostics; the public PathManager checks rejection and cache recovery directly.
The new regression failed against the baseline before the implementation change.

## Verification

Final local validation used GCC 13.3, C++17 and `-Werror`:

- Release: all 56 CTests passed (101.08 s).
- Debug: all 56 CTests passed (361.91 s).
- Both runs include the pinned official ROS Rolling format check, installed
  consumer and independent compilation of all 19 public headers, the numerical
  regressions, and existing raw/timed/profile multi-seed closed loops.
- Final official-format and `git diff --check` verification passed.

After functional tests completed, an isolated production integration-budget probe
ran 50 cold-start repetitions per obstacle count with the default 80 ms budget:

| Obstacles | p50 (ms) | p95 (ms) | Maximum (ms) | Compute timeouts | Total overruns | Other failures |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 32.7945 | 39.4321 | 46.6613 | 0 | 0 | 0 |
| 40 | 41.8007 | 42.4663 | 42.6063 | 0 | 0 | 0 |
| 128 | 62.8988 | 63.9193 | 65.0030 | 0 | 0 | 0 |

These host measurements do not establish target-machine timing guarantees. The
earlier 0.20.0 validation recorded deadline exhaustion under the 128-obstacle
workload; workload/budget calibration remains required before continuous real-time
simulation or hardware operation.
