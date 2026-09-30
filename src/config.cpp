#include "swerve_mppi/config.hpp"
#include <cmath>
#include <stdexcept>
namespace swerve_mppi {
void validate(const Config &c) {
  const double positive[] = {c.wheelbase_m,
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
                             c.confirmation_timeout_s,
                             c.dt_s,
                             c.temperature};
  for (const double value : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("positive, finite vehicle and MPPI parameters required");
    }
  }
  const double nonnegative[] = {c.control_correction_weight,
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
  if (c.steering_limit_rad < 1.5707963267948966 || c.steering_limit_rad > 3.14159265358979323846 ||
      c.steering_tolerance_rad >= c.steering_limit_rad || c.horizon_steps < 2 ||
      c.samples_per_branch < 1 || c.iterations < 1) {
    throw std::invalid_argument("MPPI horizon, sample count, and iterations invalid");
  }
}

} // namespace swerve_mppi
