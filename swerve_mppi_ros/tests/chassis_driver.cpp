#include <iomanip>
#include <iostream>
#include <string>

#include "swerve_mppi/controller.hpp"
#include "swerve_mppi_ros/control_support.hpp"
using namespace swerve_mppi_ros;
int main(int argc, char ** argv)
{
  try {
    const std::string scenario = argc > 1 ? argv[1] : "straight";
    std::vector<Pose> poses{{0, 0, 0}, {1, 0, 0}};
    if (scenario == "crab") poses = {{0, 0, 0}, {0, 1, 0}};
    if (scenario == "reverse") poses = {{0, 0, 0}, {-1, 0, 0}};
    if (scenario == "spin") poses = {{0, 0, 0}, {0, 0, 1.2}};
    if (scenario == "final_yaw") poses = {{0, 0, 0}, {1, 0, 1.2}};
    if (scenario == "curve") poses = {{0, 0, 0}, {0.5, 0.15, 0.3}, {1, 0.35, 0.3}, {1.5, 0.35, 0}};
    Config config;
    config.samples = 32;
    Controller controller(config);
    ChassisAdapter adapter;
    PathHandler path;
    path.set(poses);
    GoalChecker goal;
    std::cout << std::setprecision(17);
    double stamp;
    ChassisFeedback f;
    int mode, requested, switching, fault;
    bool complete = false;
    while (std::cin >> stamp >> f.state.pose.x >> f.state.pose.y >> f.state.pose.yaw >>
           f.state.velocity.vx >> f.state.velocity.vy >> f.state.velocity.wz >> mode >> requested >>
           switching >> f.request_id >> f.entry.vx >> f.entry.vy >> f.entry.wz >> fault) {
      f.state.mode = static_cast<Mode>(mode);
      f.state.requested_mode = static_cast<Mode>(requested);
      f.state.switching = switching;
      f.fault = fault;
      for (double & a : f.state.steering) std::cin >> a;
      for (double & s : f.state.wheel_speeds) {
        std::cin >> s;
        s *= config.wheel_radius;
      }
      adapter.observe(f);
      Input input;
      input.state = adapter.state();
      input.dt = 0.05;
      input.path = path.local(input.state.pose);
      input.goal = path.goal();
      complete = complete || goal.reached(input.state, input.goal, stamp);
      Result result;
      if (!complete && !fault) result = controller.compute(input);
      auto wire = adapter.translate(complete || fault ? Command{{}, f.state.mode} : result.command);
      std::cout << wire.command.velocity.vx << ' ' << wire.command.velocity.vy << ' '
                << wire.command.velocity.wz << ' ' << static_cast<int>(wire.command.mode) << ' '
                << wire.request_id << ' ' << wire.entry.vx << ' ' << wire.entry.vy << ' '
                << wire.entry.wz << ' ' << complete << ' ' << result.compute_ms << '\n'
                << std::flush;
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
