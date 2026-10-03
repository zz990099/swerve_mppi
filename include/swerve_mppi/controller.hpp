#pragma once
#include "swerve_mppi/optimizer.hpp"
#include <memory>
namespace swerve_mppi {
namespace detail {
class Planner;
}
// Public planning boundary: body velocity, explicit mode requests and diagnostics.
// Joint targets and execution actions stay inside prediction/execution layers.
class Controller {
public:
  explicit Controller(const Config &config,
                      std::shared_ptr<const TrajectoryValidator> validator = nullptr,
                      PlanningBudget::Now now = {});
  ~Controller();
  Controller(const Controller &);
  Controller &operator=(const Controller &);
  Controller(Controller &&) noexcept;
  Controller &operator=(Controller &&) noexcept;
  Output compute(const ControllerInput &input);
  void reset();
  TransitionPhase transition_phase() const;

private:
  std::unique_ptr<detail::Planner> planner_;
};
} // namespace swerve_mppi
