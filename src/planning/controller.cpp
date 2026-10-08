#include "swerve_mppi/planning/controller.hpp"

#include "planning/detail/planner.hpp"
namespace swerve_mppi
{
Controller::Controller(
  const Config & config, std::shared_ptr<const TrajectoryValidator> validator,
  PlanningBudget::Now now)
: planner_(std::make_unique<detail::Planner>(config, std::move(validator), std::move(now)))
{
}
Controller::~Controller() = default;
Controller::Controller(const Controller & other)
: planner_(std::make_unique<detail::Planner>(*other.planner_))
{
}
Controller & Controller::operator=(const Controller & other)
{
  if (this != &other) {
    planner_ = std::make_unique<detail::Planner>(*other.planner_);
  }
  return *this;
}
Controller::Controller(Controller &&) noexcept = default;
Controller & Controller::operator=(Controller &&) noexcept = default;
Output Controller::compute(const ControllerInput & input)
{
  const auto planned = planner_->compute(input);
  Output out;
  static_cast<PlanningDiagnostics &>(out) = planned;
  if (planned.action == detail::Action::SafeStop) {
    return out;
  }
  ChassisCommand command;
  command.mode = input.vehicle.actual_mode;
  command.target_velocity = planned.velocity_intent;
  if (planned.action == detail::Action::RequestMode) {
    if (!planned.mode_request) {
      return out;
    }
    const auto & r = *planned.mode_request;
    command.mode = r.mode;
    command.target_velocity = {};
    command.mode_request = ModeRequest{r.id, r.mode, r.entry_velocity};
  }
  out.command = command;
  return out;
}
void Controller::reset() { planner_->reset(); }
TransitionPhase Controller::transition_phase() const { return planner_->transition_phase(); }
}  // namespace swerve_mppi
