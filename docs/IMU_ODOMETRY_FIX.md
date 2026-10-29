# Independent heading feedback for lateral tracking

The 12 m recordings at c3e7800 show an unobserved heading offset of about
0.46–1.08 degrees after stationary steering/startup. Source-time aligned rotation
of wheel odometry increments using the measured truth/odometry heading difference
reproduced all six final lateral x offsets within 0.54 cm; this was offline
diagnosis, not a closed-loop improvement result.

The demo now uses a Gazebo IMU sensor at 100 Hz. A demo localization node combines
source-time-interpolated IMU orientation with wheel body translation and publishes
one consistent odom pose and TF. The original wheel-only odometry is retained on
wheel/odom with its TF disabled. The independent ground-truth observer is not an
input to localization or control. IMU and raw wheel pose are recorded separately.

Only bracketed IMU samples are used, quaternion wrap is interpolated, duplicate
source stamps do not advance integration and sensor gaps are not extrapolated.
Clock reversal clears estimator history. No corrected pose is published without
available fresh IMU data. Core actuator prediction and optimizer are unchanged.

Launch use_imu_odometry:=false restores the wheel-only demo for comparison.
The Gazebo IMU is ideal; this is not a calibrated real-hardware EKF, and heading
bias, noise, mounting, transport failures and long-run wheel slip still require
hardware-specific estimation. Wheel covariance remains conservatively unchanged.

Local checks: 18 offline recording/estimation tests; 15 chassis model/truth tests;
chassis Python formatting/lint passed. Full physical rerun is pending.
Use the same 3 rounds x 12 cases, seeds 42/43/44, translation length 12 m and
1 m/s cap. User geometry gates: path peak <=10 cm, endpoint <=20 cm, final yaw
<=5 degrees. Keep other engineering failures visible and do not merge to main
unless complete acceptance is satisfied.
