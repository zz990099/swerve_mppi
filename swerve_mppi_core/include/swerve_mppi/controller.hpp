#pragma once
#include <optional>
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
  bool retime(const Input & input, double dt, Result & result);
  void reset();

private:
  void clear_optimization();
  Result compute_tracking(const Input & input);
  struct BranchEvent
  {
    Pose origin, exit;
    Velocity alignment;
    int phase = 1;
  };
  std::optional<BranchEvent> branch_;
  std::vector<Velocity> seed(
    const Input & input, const ReferenceProfile & reference,
    const std::optional<Velocity> & intent = {}) const;
  Config config_;
  Critics critics_;
  Model model_;
  std::mt19937 random_;
  std::normal_distribution<double> normal_{0, 1};
  std::vector<Velocity> warm_;
  double warm_elapsed_ = 0;
  std::optional<Velocity> reorientation_;
};
}  // namespace swerve_mppi
