# Development

CMake is the core build entry point, located at `swerve_mppi_core/CMakeLists.txt`.
Production sources live under `swerve_mppi_core/src/<module>` and public headers
under `swerve_mppi_core/include/swerve_mppi/<module>`. Private planner types live
under `swerve_mppi_core/src/planning/detail` and are not installed. Tests and offline fixtures are
never linked into `swerve_mppi::core`.

Package-owned configuration, benchmarks, tests and the motion-probe report tool stay
inside the core directory. A copy of that directory can build, test and install on
its own. Only optional repository-wide formatting requires the root developer tools.
The root has no CMake entry point or compatibility wrapper.

The CMake project, installed config package and colcon manifest are all named
`swerve_mppi_core`. Headers and the link target remain `swerve_mppi/...` and
`swerve_mppi::core`. The install contains `share/swerve_mppi_core/package.xml`,
`share/swerve_mppi_core/default.conf` and the exported CMake package. Downstream
packages must consume these installed interfaces rather than relative source paths.

For standalone builds, configure with `cmake -S swerve_mppi_core -B build-cmake`.
For colcon, install `colcon-common-extensions` and build from the repository root:

```bash
colcon list --base-paths .
colcon build --base-paths . --packages-select swerve_mppi_core --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select swerve_mppi_core --return-code-on-test-failure
colcon test-result --all --verbose
source install/local_setup.bash
```

Do not share a build directory between these workflows. Out-of-source CMake builds
write `COLCON_IGNORE` into their build directory so installed-consumer fixtures are
not rediscovered as duplicate source packages. Colcon recognizes the core
as a plain `cmake` package and needs no ROS distribution. Add the actual ROS package
in stage 5B; it will use `ament_cmake` and declare a dependency on the core. Nav2
integration follows physical acceptance and will have its own dependency boundary.

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
The format scanner visits package source directories and fails if it finds no C++
files. CI also verifies colcon discovery, build, CTest and install-prefix consumption.

Changes that remove an interface delete it and update every current consumer;
there are no deprecated aliases or old-format decoding paths for removed executors.
Keep trajectory prediction, measured feedback and physical actuation responsibilities
explicit. Complete one roadmap stage and its acceptance checks before beginning the next.
