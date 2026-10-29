#pragma once
#include <cstddef>
#include <cstdint>
#include <swerve_chassis_core/types.hpp>
namespace swerve_mppi
{
struct Config
{
  swerve_chassis_core::Geometry geometry;
  swerve_chassis_core::Limits limits;
  // Planner limits may be lower than physical limits (e.g. Nav2 speed limit).
  double max_speed = 0.8, max_yaw_rate = 0.8;
  double model_dt = 0.1, integration_dt = 0.01;
  double steering_time_constant = 0.10;  // Gazebo steering response, checked against recordings.
  std::size_t horizon = 24, samples = 24, iterations = 1;
  double noise_linear = 0.06, noise_angular = 0.08, noise_correlation = 0.8;
  double temperature = 0.5;
  double lookahead = 0.3, approach_distance = 0.5, approach_gain = 2.0;
  double max_residual = 0.08;
  double position_scale = .03, heading_scale = .05, cross_gain = 4.;
  double path_corridor = .04, heading_corridor = .10;
  std::uint32_t seed = 42;
};
void validate(const Config & config);
}  // namespace swerve_mppi
