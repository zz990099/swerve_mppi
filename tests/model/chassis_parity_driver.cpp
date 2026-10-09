#include <iomanip>
#include <iostream>

#include "swerve_mppi/model/model.hpp"
using namespace swerve_mppi;
int main(int argc, char ** argv)
{
  Config c;
  if (argc == 3) {
    c.max_wheel_speed_mps = std::stod(argv[1]);
    c.max_wheel_accel_mps2 = std::stod(argv[2]);
  }
  DriveModel model(c);
  ChassisPrediction memory;
  bool initialized = false;
  std::cout << std::setprecision(17);
  double dt;
  while (std::cin >> dt) {
    VehicleState state;
    int mode, request;
    Control u, entry;
    std::cin >> state.stamp_ns >> mode >> u.vx >> u.vy >> u.wz >> request >> entry.vx >> entry.vy >>
      entry.wz;
    state.actual_mode = static_cast<DriveMode>(mode);
    for (double & angle : state.steering_angles) std::cin >> angle;
    for (double & speed : state.wheel_speeds) std::cin >> speed;
    state.velocity = Kinematics(c).forward(state.wheel_speeds, state.steering_angles);
    if (!initialized) {
      memory = model.seed(state);
      initialized = true;
    }
    if (request) {
      memory.phase = TransitionPhase::Braking;
      memory.alignment = model.steering_for_entry(state.actual_mode, entry, state.steering_angles);
      memory.transition_start_ns = state.stamp_ns;
      memory.aligned_since_ns = -1;
    }
    const auto next = model.step(state, u, dt, memory);
    if (!next.valid) return 1;
    memory = next.prediction;
    const int phase = memory.phase == TransitionPhase::Stable    ? 3
                      : memory.phase == TransitionPhase::Braking ? 1
                                                                 : 2;
    std::cout << phase;
    for (double a : next.steering_targets) std::cout << ' ' << a;
    for (double v : next.wheel_speed_targets) std::cout << ' ' << v;
    for (double a : memory.alignment) std::cout << ' ' << a;
    std::cout << ' ' << memory.limited_velocity.vx << ' ' << memory.limited_velocity.vy << ' '
              << memory.limited_velocity.wz << '\n';
  }
}
