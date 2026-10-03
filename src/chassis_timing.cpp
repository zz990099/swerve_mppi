#include "chassis_compiler.hpp"
#include "joint_timing.hpp"
#include "swerve_mppi/timing.hpp"
namespace swerve_mppi {
struct TimedExecutor::Impl {
  detail::ChassisCompiler compiler;
  detail::JointTimedExecutor executor;
  Impl(const Config &config, std::uint64_t session, DriveMode mode, const TimingLimits &limits,
       std::shared_ptr<const TrajectoryValidator> validator)
      : compiler(config), executor(config, session, mode, limits, std::move(validator)) {}
};
TimedExecutor::TimedExecutor(const Config &config, std::uint64_t session, DriveMode mode,
                             const TimingLimits &limits,
                             std::shared_ptr<const TrajectoryValidator> validator)
    : impl_(std::make_unique<Impl>(config, session, mode, limits, std::move(validator))) {}
TimedExecutor::~TimedExecutor() = default;
TimedExecutor::TimedExecutor(TimedExecutor &&) noexcept = default;
TimedExecutor &TimedExecutor::operator=(TimedExecutor &&) noexcept = default;
TimedExecutionResult TimedExecutor::update(const std::optional<CommandEnvelope> &command,
                                           const ControllerInput &latest, double now_s) {
  auto candidate = impl_->compiler;
  std::optional<detail::JointCommandEnvelope> compiled;
  if (command) {
    compiled.emplace(detail::JointCommandEnvelope{
        command->session_id, command->sequence, command->issued_at_s,
        candidate.compile(command->command, latest.vehicle), command->source_stamp_s,
        command->execute_at_s, command->valid_until_s, std::nullopt,
        command->source_task ? &*command->source_task : nullptr});
  }
  auto result = impl_->executor.update(compiled, latest, now_s);
  if (result.timing_error == TimingError::None &&
      result.safety_error == ExecutionSafetyError::None && !result.execution.feedback.fault)
    impl_->compiler = std::move(candidate);
  return result;
}
void TimedExecutor::reset(const VehicleState &recovered, std::uint64_t new_session) {
  impl_->executor.reset(recovered, new_session);
  impl_->compiler.reset();
}
} // namespace swerve_mppi
