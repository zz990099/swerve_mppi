# Development

CMake is the build entry point. Production sources live under `src/<module>` and
installed headers under `include/swerve_mppi/<module>`. Private planner types live
under `src/planning/detail` and are not installed. Tests and offline fixtures are
never linked into `swerve_mppi::core`.

Code and documentation use English. C++ formatting uses the unchanged official
ROS Rolling ament configuration and pinned tooling:

```bash
python3 -m pip install -r tools/requirements-format.txt
python3 tools/format.py --fix
python3 tools/format.py --check
```

`SWERVE_MPPI_BUILD_FORMAT_TESTS=ON` adds the formatting check to CTest. CI also
builds both Debug and Release with warnings as errors. Library-only and installed
consumer builds must remain independent of ROS, Gazebo, fixtures and developer tools.

Changes that remove an interface delete it and update every current consumer;
there are no deprecated aliases or old-format decoding paths for removed executors.
Keep trajectory prediction, measured feedback and physical actuation responsibilities
explicit. Complete one roadmap stage and its acceptance checks before beginning the next.
