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
  out.observation_stamp_ns = input.vehicle.stamp_ns;
  out.computed_stamp_ns = input.planning_stamp_ns;
  out.valid_until_ns = planner_->command_expiry(input.planning_stamp_ns);
  if (out.valid_until_ns < out.computed_stamp_ns) {
    out.failure_reason = FailureReason::ClockFault;
    out.control_policy = ControlPolicy::Fault;
    out.navigation_status = NavigationStatus::Fault;
    return out;
  }
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
  const auto lifetime = out.valid_until_ns >= 0 ? out.valid_until_ns : out.computed_stamp_ns;
  out.command_id = planner_->record_command(input, command, out.computed_stamp_ns, lifetime);
  if (out.command_id == 0) {
    out.command.reset();
  }
  return out;
}
bool Controller::recover(const ControllerInput & input) { return planner_->recover(input); }
void Controller::reset() { planner_->reset(); }
TransitionPhase Controller::transition_phase() const { return planner_->transition_phase(); }
}  // namespace swerve_mppi
