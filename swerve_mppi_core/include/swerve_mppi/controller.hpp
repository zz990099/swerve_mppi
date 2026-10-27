#pragma once
#include <random>

#include "swerve_mppi/critic.hpp"
#include "swerve_mppi/model.hpp"

namespace swerve_mppi
{
class Controller
{
public:
  explicit Controller(Config config = {}, Critics critics = tracking_critics());
  Result compute(const Input & input);
  void reset();

private:
  std::vector<Velocity> seed(const Input & input) const;
  Config config_;
  Critics critics_;
  Model model_;
  std::mt19937 random_;
  std::normal_distribution<double> normal_{0, 1};
  std::vector<Velocity> warm_;
  double warm_elapsed_ = 0;
};
}  // namespace swerve_mppi
