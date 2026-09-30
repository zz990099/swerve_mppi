#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/executor.hpp"
int main() {
  swerve_mppi::Controller controller(swerve_mppi::Config{});
  swerve_mppi::ModeExecutor executor(swerve_mppi::Config{});
  swerve_mppi::Output hold;
  hold.action = swerve_mppi::Action::Hold;
  auto result = executor.update(hold, {});
  return controller.compute({}).action == swerve_mppi::Action::SafeStop && result.feedback.confirmed
             ? 0
             : 1;
}
