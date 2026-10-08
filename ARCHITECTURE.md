# Architecture

`Controller::compute` is the application boundary: measured vehicle state, path
and obstacles enter; optional chassis velocity/mode intent and diagnostics leave.
ROS, chassis execution and transport protocols are external responsibilities.

| Module | Responsibility |
| --- | --- |
| `common` | Value types, validated configuration and planning budgets |
| `navigation` | Ordered path progress, local target and measured goal completion |
| `planning` | MPPI optimization, noise, critics, mode selection and request tracking |
| `model` | Kinematics, velocity rollout, transition and stopping predictions |
| `feedback` | Observation conversion and diagnostic consistency assessment |
| `safety` | Nominal trajectory collision and injected hard-constraint checks |

The optimizer/noise/model/critic separation follows the general MPPI decomposition.
Swerve mode selection adds dwell, switching cost and hysteresis. Private request
tracking freezes mode entry and waits for the chassis's measured receipt. It does
not operate steering joints or decide that the real chassis has completed a switch.

Private `Prediction`, action and mode-manager types live under `src/planning/detail`.
They are not installed. Measured `AcceptedModeRequest` remains public because the
planner needs the geometry actually accepted by the chassis. Model APIs can report
predicted wheel states; application `Output` contains no actuator target arrays.

Normal drive intents are checked through a first-command/complete-stop prediction.
Braking and pending alignment use `RolloutEngine::generate_stopping_interval`, which
preserves observed mode state and checks the predicted interval plus stopping tail.
There is no actuator profile, execution admission service or profile sampler in the
production library. These checks establish feasibility under the current nominal
model, not physical braking guarantees for Gazebo or hardware.

The model uses the independent chassis period inside each prediction interval.
`ChassisPrediction` carries limited body intent, commanded joints, frozen alignment
and dwell state separately from `VehicleState`. Rollout carries this context across
steps; a cold seed explicitly assumes command-side state equals the observation.
The controller retains an issued same-mode entry if the chassis subsequently reports
automatic realignment. Pending unconfirmed alignment has a dedicated hypothetical
continuation check; only actual measured confirmation releases the live Drive gate.
The actual accepted steering receipt remains immutable for explicit mode requests.

Prediction uses ideal sampled targets and encoder-derived motion, not Gazebo servo,
contact or slip dynamics. Stage 3 handles observation uncertainty, asynchronous time
and command-history reconciliation before this can be treated as a live integration.

The test-only `NominalChassis` in `tests/fixtures/chassis_fixture.hpp` supplies
measured acknowledgements to offline loops. It shares nominal target calculation
with DriveModel, while encoder/body integration is independently implemented in
`behavior_fixture.hpp`. The fixture is not installed or linked into the core and
is not an independent validation of chassis command generation.
The separate Python parity test invokes the actual companion `Chassis` class. A separate perturbed plant
in `motion_fixture.hpp` measures the nominal model's lag/slip/noise limitations.

Production does not build, import or execute the Gazebo repository. CI checks out
a pinned companion revision solely for the optional offline Python parity test. A future
ROS adapter belongs on the algorithm side and will communicate only through the
current body-command and chassis-state messages. See the [roadmap](docs/PREPARATION_PLAN.md).
