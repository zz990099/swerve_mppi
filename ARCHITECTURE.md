# Core architecture

This repository builds one C++17 library, `swerve_mppi::core`. The library has
no ROS dependency. The future Nav2 controller and the vehicle driver belong in
separate adapter packages; they must not be included by core headers.

## Data flow

```text
measured vehicle + local path + circle obstacles
    -> Controller::compute
    -> ModeManager (continue an active real transition, if any)
    -> Optimizer::make_branches (keep mode or make one later switch)
    -> Optimizer::optimize (Gaussian samples within each fixed branch)
    -> DriveModel / TransitionModel rollout + trajectory scoring
    -> choose the lowest-cost feasible branch
    -> ModeManager (commit a new switch if improvement exceeds hysteresis)
    -> one mode-aware action, body velocity and wheel targets
```

The modes are `DualAckermann`, `Spin`, and `Crab`. A branch fixes its discrete
mode and optional switch time; MPPI weights continuous control samples **within
that branch**. Controls from different modes are never averaged together.

## Public components

| Header | Role |
| --- | --- |
| `types.hpp` | ROS-independent poses, twists, measured vehicle state, input, actions, output, and mode enums. |
| `config.hpp` | Robot dimensions, physical limits, transition estimates, MPPI settings and cost weights. |
| `model.hpp` | `DriveModel`: per-mode feasibility, projection, wheel targets and one-step propagation. `TransitionModel`: predicted braking, steering alignment and confirmation wait. |
| `optimizer.hpp` | Discrete branches, control sampling, cost evaluation and warm starts. |
| `controller.hpp` | Stateful `ModeManager` and the public `Controller::compute()` entry point. |

`src/model.cpp` implements the drive and transition models. `src/optimizer.cpp`
also contains the current path-distance and circle-obstacle costs; these are
not separately configurable plugins yet. `src/controller.cpp` implements the
runtime output and feedback gate. `tests/core_tests.cpp` covers the physical
mode constraints, transition timing, confirmation, timeout and a lateral goal.

## Prediction versus execution

The optimizer predicts a transition with configured brake, steering and
confirmation times. Prediction is only a way to compare candidate trajectories.
At execution, `ModeManager` commits a switch, commands braking, and repeatedly
emits the target mode request while waiting. It resumes motion only when the
observed mode equals the target, `mode_confirmed` is true, and the measured
velocity is stopped. A timeout or reported mode fault latches `SafeStop` until
the application deliberately calls `reset()`.

The adapter must treat `RequestMode` as idempotent and perform the actual
wheel alignment and mode change safely. The controller does not infer a
successful switch from elapsed time or a sent request. The adapter must
provide monotonically increasing timestamps and the age of the confirmed
actual mode. It must not translate `RequestMode` into an ordinary `Twist`.

## Build and downstream use

The top-level CMake project builds `swerve_mppi_core` and optionally the
`swerve_mppi_tests` executable (`SWERVE_MPPI_BUILD_TESTS=ON`, the default).
`cmake --install` exports the `swerve_mppi::core` target and a versioned
`swerve_mppiConfig.cmake`; a separate adapter can use:

```cmake
find_package(swerve_mppi 0.1 CONFIG REQUIRED)
target_link_libraries(my_adapter PRIVATE swerve_mppi::core)
```

Set `CMAKE_PREFIX_PATH` to the install prefix. No ROS or Nav2 packages are
needed to build the core or run its tests.

## Next development boundaries

1. Replace the prototype dimensions and per-mode limits with calibrated
   steering geometry, joint limits, actuator rate limits and measured switch
   timing. Keep the actual feedback gate in `ModeManager`.
2. Replace static circle obstacles and circular footprint checks with a
   ROS-independent footprint and obstacle-query interface. A Nav2 adapter can
   then translate costmap data without leaking ROS types into this library.
3. Add measurements for solve-time distribution, closed-loop progress, mode
   switch counts and faults before choosing a target control rate or increasing
   the number of discrete branches.
4. Implement a separate `nav2_core::Controller` adapter and define its
   mode-request/acknowledgement channel with the vehicle controller. A velocity
   message alone cannot express this contract.

This is a research prototype. The current mode dynamics, costs and transition
durations are not calibrated or validated for a physical platform.
