#pragma once

#include <cstddef>
#include <cstdint>

namespace swerve_mppi {

struct Config {
  double wheelbase_m = 0.7;
  double track_m = 0.55;
  double robot_radius_m = 0.42;
  double collision_margin_m = 0.05;

  double max_wheel_speed_mps = 1.5;
  double max_steer_rate_radps = 2.0;
  double max_vx_mps = 0.8;
  double max_crab_speed_mps = 0.65;
  double max_spin_radps = 0.8;
  double max_yaw_rate_radps = 0.7;
  double min_turn_radius_m = 0.6;
  double max_linear_accel_mps2 = 0.9;
  double max_linear_decel_mps2 = 1.0;
  double max_angular_accel_radps2 = 1.3;
  double steering_tolerance_rad = 0.10;
  double stopped_linear_mps = 0.035;
  double stopped_angular_radps = 0.035;

  double minimum_mode_dwell_s = 1.0;
  double alignment_min_s = 0.25;
  double confirmation_prediction_s = 0.20;
  double confirmation_timeout_s = 2.0;
  double switch_cost = 1.0;
  double switch_hysteresis = 0.4;

  double dt_s = 0.1;
  std::size_t horizon_steps = 20;
  std::size_t samples_per_branch = 80;
  std::size_t iterations = 2;
  double temperature = 0.35;
  double noise_v_mps = 0.30;
  double noise_w_radps = 0.35;
  std::uint32_t random_seed = 42;

  double path_weight = 0.5;
  double goal_weight = 6.0;
  double yaw_weight = 0.8;
  double effort_weight = 0.015;
  double clearance_weight = 0.08;
};

void validate(const Config & config);

}  // namespace swerve_mppi
