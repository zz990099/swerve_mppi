# Performance

Enable `SWERVE_MPPI_BUILD_BENCHMARKS=ON` to build `swerve_mppi_benchmark` and
`swerve_mppi_model_probe`. The former records controller wall-time percentiles,
ordinary C++ allocation counts, work counters and nominal closed-loop behavior.
Fixture stepping is outside the compute measurement. Examples:

```bash
./build/swerve_mppi_benchmark curve 42 > curve.csv
./build/swerve_mppi_benchmark dense_curve 42 > dense-curve.csv
./build/swerve_mppi_benchmark --smoke
```

Measurements are host-dependent. The allocation smoke check is a regression gate;
wall-clock percentiles are observations, not a live scheduling guarantee. The core
still enforces its configured cooperative compute budget and rejects late results.
The former execution-envelope/profile pipeline benchmark was removed along with
that pipeline. No replacement output pretends to measure transport or actuation.

See [model feedback](MODEL_FEEDBACK.md) for the independent perturbation probe.
Record commit, build settings and host details when comparing performance results.
