#include "swerve_mppi/common/config.hpp"

#include <cmath>
#include <stdexcept>
namespace swerve_mppi
{
void validate(const Config & c)
{
  const double positive[] = {
    c.wheelbase_m,
    c.path_lookahead_m,
    c.path_lookahead_turn_rad,
    c.path_search_window_m,
    c.goal_position_tolerance_m,
    c.goal_yaw_tolerance_rad,
    c.goal_slowdown_distance_m,
    c.goal_translation_gain,
    c.goal_rotation_gain,
    c.progress_timeout_s,
    c.progress_distance_m,
    c.track_m,
    c.wheel_radius_m,
    c.robot_radius_m,
    c.max_wheel_speed_mps,
    c.max_wheel_accel_mps2,
    c.steering_limit_rad,
    c.max_steer_rate_radps,
    c.max_vx_mps,
    c.max_crab_speed_mps,
    c.max_spin_radps,
    c.max_yaw_rate_radps,
    c.min_turn_radius_m,
    c.max_linear_accel_mps2,
    c.max_linear_decel_mps2,
    c.max_angular_accel_radps2,
    c.max_angular_decel_radps2,
    c.steering_tolerance_rad,
    c.drive_steering_limit_rad,
    c.confirmation_timeout_s,
    c.dt_s,
    c.temperature};
  for (const double value : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("positive, finite vehicle and MPPI parameters required");
    }
  }
  const double nonnegative[] = {
    c.drive_kinematic_tolerance_mps,
    c.feedback_linear_tolerance_mps,
    c.feedback_angular_tolerance_radps,
    c.compute_budget_ratio,
    c.control_correction_weight,
    c.path_progress_slack_m,
    c.goal_settle_time_s,
    c.path_heading_weight,
    c.smoothness_weight,
    c.goal_speed_weight,
    c.stopped_wheel_speed_mps,
    c.collision_margin_m,
    c.stopped_linear_mps,
    c.stopped_angular_radps,
    c.minimum_mode_dwell_s,
    c.alignment_min_s,
    c.confirmation_prediction_s,
    c.switch_cost,
    c.switch_hysteresis,
    c.noise_v_mps,
    c.noise_w_radps,
    c.path_weight,
    c.goal_weight,
    c.yaw_weight,
    c.effort_weight,
    c.clearance_weight};
  for (const double value : nonnegative) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("nonnegative, finite parameters required");
    }
  }
  if (
    c.steering_limit_rad < 1.5707963267948966 || c.steering_limit_rad > 3.14159265358979323846 ||
    c.goal_docking_distance_m <= c.goal_position_tolerance_m ||
    c.goal_docking_distance_m > c.goal_slowdown_distance_m ||
    !std::isfinite(c.goal_docking_distance_m) ||
    c.goal_yaw_tolerance_rad >= 3.14159265358979323846 ||
    c.path_lookahead_turn_rad >= 3.14159265358979323846 || !std::isfinite(c.noise_correlation) ||
    c.noise_correlation < 0.0 || c.noise_correlation >= 1.0 ||
    c.steering_tolerance_rad >= c.steering_limit_rad ||
    c.drive_steering_limit_rad < c.steering_tolerance_rad ||
    c.drive_steering_limit_rad > c.steering_limit_rad || c.horizon_steps < 2 ||
    c.horizon_steps > 512 || c.stopping_horizon_steps > 4096 || c.samples_per_branch > 2048 ||
    c.iterations > 32 || c.compute_budget_ratio > 1 || c.max_path_points < 1 ||
    c.max_path_points > 65536 || c.max_obstacles < 1 || c.max_obstacles > 4096 ||
    c.stopping_horizon_steps < 2 || c.safety_reduction_attempts > 16 || c.samples_per_branch < 1 ||
    c.iterations < 1) {
    throw std::invalid_argument("MPPI horizon, sample count, and iterations invalid");
  }
}

}  // namespace swerve_mppi
