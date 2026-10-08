#pragma once
#include <memory>

#include "swerve_mppi/common/config.hpp"
#include "swerve_mppi/common/planning_budget.hpp"
#include "swerve_mppi/common/types.hpp"
namespace swerve_mppi
{
class TrajectoryValidator;
namespace detail
{
class Planner;
}
// Public planning boundary: body velocity, explicit mode requests and
// diagnostics. Joint targets and execution actions stay inside
// private prediction code.
class Controller
{
public:
  explicit Controller(
    const Config & config, std::shared_ptr<const TrajectoryValidator> validator = nullptr,
    PlanningBudget::Now now = {});
  ~Controller();
  Controller(const Controller &);
  Controller & operator=(const Controller &);
  Controller(Controller &&) noexcept;
  Controller & operator=(Controller &&) noexcept;
  Output compute(const ControllerInput & input);
  void reset();
  TransitionPhase transition_phase() const;

private:
  std::unique_ptr<detail::Planner> planner_;
};
}  // namespace swerve_mppi
