# Model and feedback preparation (0.21.3)

The current core uses a nominal encoder model. `VehicleState.velocity` must agree
with wheel/steering forward kinematics within 1e-9 to admit nominal prediction,
complete-stop certification and execution. `check_feedback` has wider diagnostic
tolerances (default 0.05 m/s and 0.10 rad/s); passing those diagnostics never grants
model admission. The thresholds and all existing execution gates remain unchanged.

`FeedbackAdapter` derives velocity from the same joint observation. That provides
consistent units and timing, but body/encoder agreement is tautological in this
path. Encoder-derived odometry is not independent localization. Exact agreement
cannot detect tire slip, chassis coasting with stopped wheels, actuator response
lag, common calibration error or a displaced pose.

## Independent motion observation

The additive `feedback/motion_observer.hpp` API evaluates a sidecar observation.
It does not change `VehicleState`, command layout, model equations or executor
ownership. Construct `MotionObserver` from the resolved execution configuration;
configuration validation occurs at construction. Call `assess(state,
original_encoder_stamp_ns, observation)` with a coherent snapshot.

`MotionObservation` contains body-frame twist, the original integer timestamp,
explicit source provenance, and nonnegative finite deterministic linear/angular
error bounds. A linear bound encloses the Euclidean velocity-vector error; an
angular bound encloses absolute yaw-rate error. These are not standard deviations,
covariances or probabilistic confidence intervals. Bounds must be justified by
independent calibration/estimation; the API cannot authenticate provenance or
prove the bounds. Zero means a declared exact observation, appropriate to the
synthetic oracle, not an assumption to copy into a noisy sensor adapter.

Both original integer timestamps must match exactly. `state.stamp_s` must equal
one conversion of the encoder timestamp using `duration<double>`. Never regenerate
integer stamps from doubles. Freshness does not establish coherence, and delayed
body velocity must not be combined with current joints or relabeled as current.
The observer provides neither extrapolation nor freshness/watchdog enforcement.

For encoder velocity e, observed body velocity b, and error bound u, the
conservative residual upper bound is `norm(b-e)+u`. Body-motion upper bounds are
`norm(b)+u`. Linear norms are Euclidean; angular norms are absolute value. This
bounds body-observation uncertainty relative to the supplied encoders only.
Encoder errors, pose uncertainty, future slip and braking response uncertainty
are not enclosed. Nonrepresentable arithmetic fails closed as Invalid.

| Assessment | Meaning | Nominal-only owner policy for a future independent acceptance path |
| --- | --- | --- |
| Invalid / Missing | Invalid context, timestamp, value/bound/provenance, or no observation | Withhold normal command/profile installation; invoke external stop/latch ownership |
| Unsynchronized | Original body and encoder stamps differ | Withhold; reacquire a coherent snapshot; never rewrite stamps |
| CorrelatedSource | Observation was derived from encoders | Withhold independent acceptance; retain only nominal development evidence |
| NominalAgreement | Independent residual <=1e-9, zero declared uncertainty, and diagnostic envelope satisfied | Necessary evidence only; all existing model, geometry, timing, mode and profile gates still apply |
| BoundedDisagreement | Residual plus uncertainty fits the diagnostic envelope | Record diagnostics; withhold nominal authorization; do not widen the 1e-9 model gate |
| EnvelopeExceeded | A residual upper bound exceeds a diagnostic threshold | Record fault evidence and stop/latch; do not turn thresholds into slip compensation |

Envelope boundaries are inclusive without adding a numerical epsilon to declared
bounds. Nonzero uncertainty is never NominalAgreement, even with zero measured
residual. `body_stationary` and `encoder_stationary` are separate instantaneous
threshold facts, not a combined recovery/transition permission. Invalid, missing,
correlated and unsynchronized assessments expose neither stationary evidence nor
finite residual bounds. A valid observation can show body stationary while wheels
roll, or wheels stationary while the body moves.

This table specifies the conservative policy for later adapter work. The observer
is stateless and diagnostic: it is not automatically wired into Controller,
TimedExecutor or ProfileRunner. Existing encoder-only callers retain their current
nominal behavior. A body residual actually supplied through VehicleState is still
rejected by the existing strict gates. Checking a sidecar and continuing to drive
with encoder-only state would not enforce this policy.

## Fault, stop and mode ownership

A future independent acceptance owner must inhibit new Drive/align/request
installation when independent evidence is unavailable or disagrees, revoke any
previous active actuator profile through its stop channel, and latch the fault.
Calling only Controller or observing an absent command does not physically stop
an already running actuator callback. When nominal feedback is inconsistent,
the core withholds a checked profile; an external emergency/controlled-stop
mechanism owns actual braking. A nominal stopping trajectory cannot certify an
unmodelled plant's braking distance.

Do not confirm a new mode, steer for alignment, declare navigation completion or
reset from encoder-only stopped feedback. Require independently bounded body
motion, stopped wheels, coherent current mode feedback and sustained fresh
observations for the configured dwell/settling contract. Instantaneous observer
booleans alone do not prove dwell. Recovery must verify plant stop, discard queued
commands/profiles, explicitly reset the owner and renew the execution session.
Current core reset still requires nominal stopped feedback; bounded disagreement
has no implemented uncertainty-aware recovery exception.

To accept noisy/slipping motion later, implement and review a robust model first:
include initial pose/twist/joint uncertainty, actuator lag and minimum braking
capability in swept collision and complete-stop envelopes; propagate uncertainty
through transitions; establish independent stop/dwell/recovery rules; validate on
an identified plant. Merely increasing `feedback_*_tolerance` cannot achieve this.
This preparation stage deliberately retains nominal-only model admission.

## Reproduce the offline matrix

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSWERVE_MPPI_BUILD_BENCHMARKS=ON
cmake --build build --parallel 2
mkdir -p build/motion-probe
build/swerve_mppi_model_probe --print-config > build/motion-probe/resolved.conf
build/swerve_mppi_model_probe --config build/motion-probe/resolved.conf > build/motion-probe/calls.csv
python tools/summarize_motion_probe.py build/motion-probe/calls.csv > build/motion-probe/report.md
```

The CSV schema version is 1. It contains 720 rows: Ackermann straight translation,
Crab lateral translation and Spin yaw motion, each with six perturbations and
20 Drive plus 20 Brake ticks. Each prediction starts from the actual independent
plant pose and current encoder state. Errors compare the core's next predicted
pose against independent plant integration over one tick; these are not accumulated
open-loop trajectory errors or controller tracking errors. The fixed Drive intents
are 0.4 m/s, 0.4 m/s and 0.5 rad/s respectively. Profiles incompatible with these
intents fail without a partial CSV. Probe periods must be in [0.001,1] seconds and
represent an exact integer-nanosecond interval.

| Perturbation | Synthetic assumption |
| --- | --- |
| nominal | Affine Drive joint targets and phased proportional braking |
| wheel_lag | First-order wheel response, time constant 0.15 s, otherwise nominal body kinematics |
| body_lag | First-order body response, time constant 0.20 s, including post-wheel-stop coasting |
| slip | Physical translation/yaw speed is 75% of encoder kinematics |
| noise | Alternating bounded body measurement error: 0.005 m/s along body x and 0.01 rad/s in yaw |
| delayed | Previous tick's complete body observation with its original stamp retained |

The numerical plant uses 512 substeps per tick and its own module-vector sums and
world-pose integration. Core model endpoints are actuator targets only; no core
FK, rollout, transition model or predicted pose/twist advances the plant. Pose is
assumed exactly observed in this probe. Noise is measurement perturbation, separate
from MPPI proposal noise. Cases are isolated deterministic experiments; combined
faults, transport jitter, steering lag, biased encoders and physical identification
remain additional validation work.

Regression oracles cover analytical ramp distance, slip travel, first-order ramp
response and refinement convergence; the CSV checker independently reconstructs
residuals/error bounds from serialized observations. Mode/stop tests exercise strict
residual rejection, no profile, no committed request, persistent faults and explicit
session recovery. CI records the complete CSV, resolved configuration, report,
binary/config/CSV digests, CMake build settings and checkout SHA in a `motion-probe` artifact.

Default offline observations: wheel lag can retain NominalAgreement while the
next-pose prediction is wrong; body lag produces stopped-wheel/moving-body samples;
bounded noise remains inadmissible to nominal models; delayed observations are
rejected as unsynchronized. These demonstrate missing assumptions, not identified
physical bounds or Gazebo acceptance. No simulator or ROS code changes here.
