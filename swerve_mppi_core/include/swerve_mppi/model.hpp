#pragma once
#include <swerve_chassis_core/setpoint_generator.hpp>

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/types.hpp"
namespace swerve_mppi
{
// Nominal low-speed model. Shared target generation does not imply a tire model.
class Model
{
public:
  explicit Model(Config config);
  Velocity condition(
    Velocity desired, Velocity previous, const std::array<double, 4> & reference, double dt) const;
  Velocity project(Velocity velocity) const;
  bool valid(const State & state) const;
  Trajectory rollout(const State & initial, const std::vector<Velocity> & controls) const;

private:
  Config config_;
  swerve_chassis_core::Generator generator_;
};
}  // namespace swerve_mppi
