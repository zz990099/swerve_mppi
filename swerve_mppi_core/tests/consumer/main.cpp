#include "swerve_mppi/controller.hpp"
int main()
{
  swerve_mppi::Controller controller;
  swerve_mppi::Input input;
  input.path = {{0, 0, 0}, {1, 0, 0}};
  input.goal = input.path.back();
  return controller.compute(input).status == swerve_mppi::Status::Ok ? 0 : 1;
}
