# Configurable dynamics and safety contracts (0.9.0)

Review baseline: 220bfb77f8e64f3090e0e447eccfd0705e6e1c41 (0.8.0).
Recorded on 2026-10-01. No ROS, simulator or hardware integration was added.

## Corrections

### Signed velocity changes

DriveModel splits the linear velocity segment at its minimum norm and the signed
angular segment at zero. It budgets braking travel with the configured deceleration
before using remaining time for acceleration. Wheel acceleration and steering-rate
limits still reduce the common joint interpolation fraction. The encoder-derived
result is rechecked against the same time budget, including the steering change.
This retains normal braking performance when deceleration exceeds acceleration;
it does not forbid either legal ordering of those parameters.

The signed-speed oracle in model_tests calculates time to zero plus acceleration
time independently. It covers both signs, Ackermann longitudinal travel, Crab
lateral travel and Spin yaw, two rate orderings, two initial/target magnitudes and
three periods (144 cases). A controller reverse-capture regression starts at
+0.03 m/s with deceleration 0.1 m/s^2; its first 0.1 s command remains at least
+0.02 m/s rather than immediately reversing.

### Injected validator compatibility

TrajectoryValidator::require_compatible rejects different robot_radius_m,
collision_margin_m, steering_limit_rad or max_wheel_speed_mps. These are the Config
fields its base collision and measured-input checks consume. Controller,
CriticManager and therefore standalone Optimizer reject mismatches during
construction with invalid_argument, before any motion is computed. Configuration
comparison is exact; planning-only weights, noise and seeds may differ because
validation consumes explicit poses. Additional TrajectoryConstraint objects remain
the supported way to extend hard safety checks. Build them before sharing a const
validator and serialize controller/optimizer calls as before.

Tests reject each incompatible field through all three public consumers, allow
planning-only differences and retain injected-constraint regressions for tracking,
capture, alignment and stationary/pending-request stopping. The smaller-footprint
capture reproduction can no longer construct an inconsistent controller.

### Individual module velocity compatibility

Kinematics::max_module_residual returns the largest Euclidean error between each
signed rolling vector and the rigid-body field (vx - wz*y_i, vy + wz*x_i).
ModeExecutor checks this against drive_kinematic_tolerance_mps, default 0.02 m/s;
DriveModel uses the same envelope for ready, nonzero Drive interpolation. Numerical
comparison tolerance is 1e-9 m/s. Nonfinite residual inputs return infinity.
Pure braking predictions retain measured steering and do not impose this Drive
residual envelope on measured slip/inconsistent pairs.

Tests reject counteracting wheels whose least-squares twist looks consistent,
check acceptance/rejection on either side of the finite residual allowance, and
permit ideal commands with zero tolerance. Existing continuous steering, curved
Ackermann, Spin and default noisy closed loops remain required. This bounds target
compatibility, not calibrated tire slip or hardware tracking.

### Confirmation and handover cycles

Prediction reserves max(2, ceil(confirmation_prediction_s / dt_s)) post-alignment
stopped cycles. The two mandatory cycles represent executor confirmation Hold and
manager measured-feedback handover Hold. Larger allowances reserve additional
latency; zero or short allowances cannot shorten the protocol. The deadline is
checked at confirmation receipt, one tick before first Drive; the entire stopped
handover must still fit the horizon. Execution feedback is never synthesized.

The execution matrix compares predicted first motion against ModeManager and
ModeExecutor driven by the independent encoder fixture. It covers three periods,
five post-alignment allowances, two alignment minima, two steering rates, all six
directed transitions and immediate/future switches: 720 stopped-entry cases.
Larger allowances are exercised with deliberately delayed confirmation feedback.
The zero-delay diagonal Controller capture reproduction now predicts and executes
first Drive at tick 3. Separate horizon and receipt-deadline edge tests ensure a
one-tick horizon is rejected while an on-time receipt may finish handover afterward.
Existing moving-entry braking and transition fault regressions remain in place.

## Verification

GNU 13.3.0, CMake 4.4.3, C++17, x86_64 Linux; -Werror. Validation used the
final 0.9 source and package changes shipped with this report.

| Check | Result |
| --- | --- |
| Release, benchmark enabled | 22/22 CTest entries passed |
| Debug, benchmark enabled | 22/22 CTest entries passed |
| AddressSanitizer + UndefinedBehaviorSanitizer | Seven core suites passed |
| Behavior matrix | Thirteen scenarios, five seeds each, both build types |
| Installed consumer | Finds 0.9 and exercises validator compatibility/module residual APIs |
| Allocation smoke | Existing straight/seed-42 peak-at-most-200 gate passed |
| Formatting and whitespace | clang-format and git diff --check passed |

The sanitizer build uses the existing non-PIE setup and detect_leaks=0; behavior,
installed-consumer and benchmark entries are excluded from its seven-suite run.
The optimizer executable initially lacked local execute permission. Restoring
that permission and rerunning the complete seven-suite matrix passed; no source
workaround was needed and no leak-detection claim is made.

A preliminary conservative shared-rate limit caused one lateral seed to miss its
completion budget. The final implementation uses the braking/acceleration time
split described above, retaining configured braking performance. All existing
completion/corridor criteria and random seeds are unchanged and pass. No new
wall-clock performance matrix was recorded; historical timing remains versioned.

## Migration and limits

The public Config layout and Kinematics/TrajectoryValidator APIs changed. Rebuild
all consumers with find_package(swerve_mppi 0.9 CONFIG REQUIRED). The CMake package
continues to require the same minor version. Tune the module residual allowance
from independent target/encoder evidence rather than disabling this check to hide
incompatible modules. This release retains circular static obstacles, one-switch
horizons, seeded mode-entry geometry and no solver deadline. The independent
encoder fixtures are protocol/behavior test utilities, not a calibrated plant.
Historical 0.8 timing CSVs remain unchanged and do not certify 0.9 timing.
