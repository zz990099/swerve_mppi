#include "swerve_mppi/chassis_executor.hpp"
#include "chassis_compiler.hpp"
namespace swerve_mppi {
struct ChassisExecutor::Impl {
  detail::ChassisCompiler compiler;
  ModeExecutor executor;
  Impl(const Config &config, DriveMode mode) : compiler(config), executor(config, mode) {}
};
ChassisExecutor::ChassisExecutor(const Config &config, DriveMode mode)
    : impl_(std::make_unique<Impl>(config, mode)) {}
ChassisExecutor::~ChassisExecutor() = default;
ChassisExecutor::ChassisExecutor(ChassisExecutor &&) noexcept = default;
ChassisExecutor &ChassisExecutor::operator=(ChassisExecutor &&) noexcept = default;
ExecutionResult ChassisExecutor::update(const Output &command, const VehicleState &measured) {
  auto candidate = impl_->compiler;
  const auto execution = impl_->executor.update(candidate.compile(command, measured), measured);
  if (!execution.feedback.fault)
    impl_->compiler = std::move(candidate);
  return execution;
}
void ChassisExecutor::reset(const VehicleState &recovered) {
  impl_->executor.reset(recovered);
  impl_->compiler.reset();
}
} // namespace swerve_mppi
