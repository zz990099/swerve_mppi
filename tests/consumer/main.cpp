#include "swerve_mppi/controller.hpp"
int main() {
  swerve_mppi::Controller controller(swerve_mppi::Config{});
  return controller.compute({}).action == swerve_mppi::Action::SafeStop ? 0 : 1;
}
