# Module layout and ROS format (0.19)

The core has seven functional modules.
Public headers live in `include/swerve_mppi/<module>/`; implementations and private
headers live in `src/<module>/` and `src/<module>/detail/`.
Module directories organize responsibilities within one `swerve_mppi::core` library;
they do not create independently linked libraries or change the public namespace.

| Module | Responsibilities and representative headers |
| --- | --- |
| `common` | Configuration, shared value types, planning budget; `config.hpp`, `types.hpp`, `planning_budget.hpp`. |
| `model` | Kinematics, drive/transition prediction, actuator profiles, rollout; `model.hpp`, `actuation.hpp`, `rollout.hpp`. |
| `navigation` | Ordered path progress, corner capture, goal completion; `navigation.hpp`. |
| `planning` | Controller orchestration, MPPI optimization, critics, noise, mode selection; `controller.hpp`, `optimizer.hpp`, `mode.hpp`. |
| `execution` | Body command compilation, measured mode supervision, timing/task admission, profile consumption; `chassis_executor.hpp`, `executor.hpp`, `timing.hpp`, `profile_runner.hpp`. |
| `feedback` | Body/encoder admission and named coherent observation conversion; `feedback.hpp`, `feedback_adapter.hpp`. |
| `safety` | Hard trajectory constraints and stopping/collision validity; `trajectory_validator.hpp`. |

Each source module owns its explicit `CMakeLists.txt` source manifest.
`src/CMakeLists.txt` assembles these modules into the existing core target.
Private includes use qualified paths such as `model/detail/motion_profile.hpp`;
the source include root is PRIVATE and is never exported or installed.
Tests mirror their functional module, with shared encoder/profile oracles in
`tests/fixtures/` and whole-pipeline behavior in `tests/integration/`.
Benchmarks remain separate measurement programs.

`ExecutionResult` is a shared value type in `common/types.hpp`.
The model's actuator profile consumes that value without importing an executor
implementation.
Controller's public header exposes configuration, value types and the budget hook;
it forward-declares its validator and keeps optimizer/workspace dependencies private.
Feedback admission and the model still cooperate on encoder consistency; directories
are organizational boundaries, not assertions that every runtime dependency is acyclic.

## Header migration

Version 0.19 changes public include paths; downstream sources must update them.
For example:

```cpp
#include <swerve_mppi/common/config.hpp>
#include <swerve_mppi/planning/controller.hpp>
#include <swerve_mppi/execution/timing.hpp>
#include <swerve_mppi/execution/profile_runner.hpp>
```

```cmake
find_package(swerve_mppi 0.19 CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE swerve_mppi::core)
```

Use the module table above to map each old flat header to its new directory.
There are no flat forwarding headers.
Classes, methods, the chassis command contract and the exported core target retain
their behavior.
The independent installed consumer compiles every installed public header in its own
translation unit, then links/runs without access to private source headers.

## Official ROS Rolling format

The formatting authority is the upstream ROS 2 Rolling `ament_clang_format`
configuration, copied byte-for-byte into `.clang-format`.
It is one of the tools listed in the official
[Rolling code style guide](https://github.com/ros2/ros2_documentation/blob/rolling/source/The-ROS2-Project/Contributing/Contributing-to-code/Code-Style-Language-Versions.rst).
The formatter remains Clang; the style and checking entry point are official ament.
No local formatting overrides are maintained.

`tools/requirements-format.txt` pins ament_lint/ament_clang_format 0.21.3 to
[upstream commit 8d5a624](https://github.com/ament/ament_lint/tree/8d5a6246a6620a4f548aa485c87b271f0692bd42),
plus clang-format 21.1.8 and PyYAML 6.0.3.
This is the Rolling snapshot verified on 2026-10-03.
Rolling can change; updating this baseline is a deliberate dependency/config update
followed by a complete reformat and regression run.
Formatting follows Rolling while the portable library retains C++17 and independent
CMake builds; ROS runtime packages are not required to compile or consume the core.

Install development tools into a virtual environment:

```bash
python3 -m venv /tmp/swerve-format-env
source /tmp/swerve-format-env/bin/activate
python -m pip install -r tools/requirements-format.txt
python tools/format.py --fix
python tools/format.py --check
```

The script invokes upstream ament_clang_format over all C++ headers/sources in
include, src, tests and benchmarks.
It rejects an incorrect formatter version or any divergence between the repository
configuration and the installed pinned upstream configuration.
`--fix` applies the official format and runs a final idempotence check.
The default/`--check` mode fails on any style divergence.
Editors should use the repository `.clang-format` with the same formatter version.

ROS conventions include two-space indentation, 100-character lines, middle pointer
spacing and open function/class/namespace braces.
Control-flow statements in the migrated code use explicit braces, including single
statements, as required by the ROS guide.
Code review still owns language/design conventions that a formatter cannot enforce;
the gate is formatting verification, not a substitute for semantic review or a
claim that all Google/ROS lint categories have been enabled.

CMake exposes `format` and `format_check` targets when Python is available.
Enable the CTest gate for development/CI:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DSWERVE_MPPI_BUILD_BENCHMARKS=ON \
  -DSWERVE_MPPI_BUILD_FORMAT_TESTS=ON \
  -DPython3_EXECUTABLE="$(command -v python)"
cmake --build build --target format_check
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure --parallel 2
```

CI installs the same pinned tools, checks formatting before compilation and includes
`ros_format` in Debug and Release CTest runs.
Ordinary core builds leave the developer-format test disabled by default.
