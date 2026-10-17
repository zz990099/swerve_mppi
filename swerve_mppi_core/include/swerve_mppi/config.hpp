#pragma once
#include <cstddef>
#include <cstdint>

namespace swerve_mppi
{
struct Config
{
  double wheelbase = 0.6, track = 0.5, wheel_radius = 0.1;
  double max_speed = 0.8, max_yaw_rate = 0.8, min_turn_radius = 0.6;
  double linear_accel = 0.9, angular_accel = 1.3;
  double max_wheel_speed = 2.0, wheel_accel = 4.0;
  double steer_rate = 2.5, steer_limit = 1.5707963267948966;
  double realign_angle = 0.2, alignment_tolerance = 0.05, alignment_dwell = 0.05;
  double stopped_wheel_speed = 0.005;
  double model_dt = 0.1, integration_dt = 0.01;
  std::size_t horizon = 30, samples = 64, iterations = 1;
  double noise_linear = 0.18, noise_angular = 0.25, noise_correlation = 0.8;
  double temperature = 0.3;
  double minimum_mode_dwell = 1.0, switch_penalty = 0.3, switch_hysteresis = 0.15;
  double lookahead = 0.6, approach_distance = 0.5, approach_gain = 1.2;
  std::uint32_t seed = 42;
};
void validate(const Config & config);
}  // namespace swerve_mppi
