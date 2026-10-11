#pragma once

#include <cstddef>
#include <cstdint>

namespace swerve_mppi
{

struct Config
{
  double wheelbase_m = 0.6;
  double track_m = 0.5;
  double wheel_radius_m = 0.1;
  double robot_radius_m = 0.5;
  double collision_margin_m = 0.05;

  // Linear rolling units; adapters convert joint rad/s using wheel_radius_m.
  double max_wheel_speed_mps = 2.0;
  double max_wheel_accel_mps2 = 4.0;
  // Current chassis mechanical stops: exactly +/- pi/2.
  double steering_limit_rad = 1.5707963267948966;
  double max_steer_rate_radps = 2.5;
  double chassis_max_linear_speed_mps = 0.8;
  double chassis_max_angular_speed_radps = 0.8;
  // Prediction substep at the independent chassis update period.
  double chassis_period_s = 0.01;
  // Mode-specific planner proposal caps, possibly stricter than the chassis.
  double max_vx_mps = 0.8;
  double max_crab_speed_mps = 0.65;
  double max_spin_radps = 0.8;
  double max_yaw_rate_radps = 0.7;
  double min_turn_radius_m = 0.6;
  double max_linear_accel_mps2 = 0.9;
  // Capture slowdown policy only; normal zero brakes each wheel independently.
  double capture_linear_decel_mps2 = 1.0;
  double max_angular_accel_radps2 = 1.3;
  double capture_angular_decel_radps2 = 1.3;
  double steering_tolerance_rad = 0.05;
  // Maximum target joint change allowed without stopping in a stable mode.
  // Set equal to steering_tolerance_rad for a conservative stop/align policy.
  double drive_steering_limit_rad = 0.20;
  double stopped_linear_mps = 0.035;
  double stopped_angular_radps = 0.035;
  double stopped_wheel_speed_mps = 0.005;
  // Absolute disagreement between body-frame twist and wheel forward
  // kinematics. Zero requests numerical agreement; these tolerances do not
  // model tire slip.
  double feedback_linear_tolerance_mps = 0.05;
  double feedback_angular_tolerance_radps = 0.10;

  // Age of the confirmed actual mode, not request stability time.
  double minimum_mode_dwell_s = 1.0;
  double alignment_min_s = 0.05;
  // Total post-alignment allowance; prediction enforces at least two protocol
  // ticks.
  double confirmation_prediction_s = 0.20;
  // Planner waiting deadline includes braking, alignment and acknowledgement.
  double confirmation_timeout_s = 5.0;
  double switch_cost = 1.0;
  double switch_hysteresis = 0.4;

  // Prediction discretization. This does not define callback frequency.
  double model_period_s = 0.1;
  // Expected interval between planning decisions.
  double planning_period_s = 0.1;
  // Fraction of planning_period_s allowed for compute, measured by steady_clock.
  // Zero disables the clock budget for offline deterministic validation only.
  double compute_budget_ratio = 0.8;
  // Source observations may be paired when their stamps differ by at most this
  // amount, then are represented by the newest source stamp.
  double observation_pairing_tolerance_s = 0.005;
  double max_observation_age_s = 0.15;
  double max_observation_gap_s = 0.30;
  double future_observation_tolerance_s = 0.002;
  double command_lifetime_s = 0.20;
  double max_command_application_uncertainty_s = 0.02;
  // Admission bounds for reconciling the preceding command prediction with
  // the next measured joint observation.
  double history_steering_tolerance_rad = 0.08;
  double history_wheel_tolerance_mps = 0.08;
  // Reject oversize contexts before scanning them. Never truncate
  // obstacles/path.
  std::size_t max_path_points = 4096;
  std::size_t max_obstacles = 128;
  std::size_t horizon_steps = 20;
  // Independent bounded budget for alignment, first Drive and a complete stop.
  // Exhaustion rejects the continuation; it never implies a stopped state.
  std::size_t stopping_horizon_steps = 200;
  // Halve a rejected intent at most this many times, preserving entry geometry.
  // Zero disables reduction. Each attempt uses the complete stopping validator.
  std::size_t safety_reduction_attempts = 8;
  std::size_t samples_per_branch = 80;
  std::size_t iterations = 2;
  double temperature = 0.35;
  double noise_v_mps = 0.30;
  double noise_w_radps = 0.35;
  // Stationary AR(1) proposals; zero restores independent time-step noise.
  double noise_correlation = 0.85;
  // Used for proposal weighting only; branch ranking uses physical critic
  // costs.
  double control_correction_weight = 0.015;
  std::uint32_t random_seed = 42;

  double path_weight = 0.5;
  double goal_weight = 6.0;
  double yaw_weight = 0.8;
  double effort_weight = 0.015;
  double clearance_weight = 0.08;
  double path_heading_weight = 0.3;
  double smoothness_weight = 0.08;
  double goal_speed_weight = 2.0;

  double path_lookahead_m = 1.0;
  double path_lookahead_turn_rad = 1.0;
  double path_search_window_m = 1.5;
  double path_progress_slack_m = 0.1;
  double goal_position_tolerance_m = 0.06;
  double goal_yaw_tolerance_rad = 0.05;
  double goal_settle_time_s = 0.3;
  double goal_slowdown_distance_m = 0.6;
  double goal_docking_distance_m = 0.25;
  double goal_translation_gain = 1.2;
  double goal_rotation_gain = 1.5;
  double progress_timeout_s = 3.0;
  double progress_distance_m = 0.03;
};

void validate(const Config & config);

}  // namespace swerve_mppi
