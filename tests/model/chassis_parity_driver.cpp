#include <iomanip>
#include <iostream>
#include <string>

#include "swerve_mppi/model/model.hpp"
using namespace swerve_mppi;
int main(int argc, char ** argv)
{
  Config c;
  if (argc == 2 && std::string(argv[1]) == "--print-interface") {
    std::cout << std::setprecision(17) << "wheelbase_m=" << c.wheelbase_m << '\n'
              << "track_m=" << c.track_m << '\n'
              << "wheel_radius_m=" << c.wheel_radius_m << '\n'
              << "chassis_period_s=" << c.chassis_period_s << '\n'
              << "max_wheel_speed_mps=" << c.max_wheel_speed_mps << '\n'
              << "max_wheel_accel_mps2=" << c.max_wheel_accel_mps2 << '\n'
              << "max_steer_rate_radps=" << c.max_steer_rate_radps << '\n'
              << "steering_limit_rad=" << c.steering_limit_rad << '\n'
              << "chassis_max_linear_speed_mps=" << c.chassis_max_linear_speed_mps << '\n'
              << "chassis_max_angular_speed_radps=" << c.chassis_max_angular_speed_radps << '\n'
              << "max_linear_accel_mps2=" << c.max_linear_accel_mps2 << '\n'
              << "max_angular_accel_radps2=" << c.max_angular_accel_radps2 << '\n'
              << "drive_steering_limit_rad=" << c.drive_steering_limit_rad << '\n'
              << "steering_tolerance_rad=" << c.steering_tolerance_rad << '\n'
              << "alignment_min_s=" << c.alignment_min_s << '\n'
              << "confirmation_timeout_s=" << c.confirmation_timeout_s << '\n'
              << "stopped_wheel_speed_mps=" << c.stopped_wheel_speed_mps << '\n'
              << "command_lifetime_s=" << c.command_lifetime_s << '\n';
    return 0;
  }
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
