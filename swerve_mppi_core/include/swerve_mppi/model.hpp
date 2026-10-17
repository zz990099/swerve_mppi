#pragma once
#include "swerve_mppi/config.hpp"
#include "swerve_mppi/types.hpp"

namespace swerve_mppi
{
struct Wheels
{
  std::array<double, 4> speeds{}, angles{};
};
// Nominal actuator response, initialized from measurement for each rollout.
// It is not a replay of a chassis executor or an application receipt.
class Model
{
public:
  explicit Model(Config config);
  Velocity project(Velocity velocity, Mode mode) const;
  Wheels inverse(Velocity velocity, const std::array<double, 4> & current) const;
  Velocity forward(const Wheels & wheels) const;
  Trajectory rollout(
    const State & initial, Mode mode, const std::vector<Velocity> & controls) const;

private:
  Config config_;
};
}  // namespace swerve_mppi
