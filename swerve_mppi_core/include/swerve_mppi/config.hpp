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
  std::size_t horizon = 30, samples = 64, iterations = 1;
  double noise_linear = 0.18, noise_angular = 0.25, noise_correlation = 0.8;
  double temperature = 0.3;
  double lookahead = 0.6, approach_distance = 0.5, approach_gain = 1.2;
  double max_residual = 0.08;
  std::uint32_t seed = 42;
};
void validate(const Config & config);
}
