#pragma once
#include "swerve_mppi/config.hpp"
#include "swerve_mppi/types.hpp"
#include <swerve_chassis_core/setpoint_generator.hpp>
namespace swerve_mppi
{
// Nominal low-speed model. Shared target generation does not imply a tire model.
class Model
{
public:
  explicit Model(Config config);
  Velocity project(Velocity velocity) const;
  bool valid(const State & state) const;
  Trajectory rollout(const State & initial, const std::vector<Velocity> & controls) const;
private:
  Config config_;
  swerve_chassis_core::Generator generator_;
};
}
