#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "swerve_mppi/controller.hpp"
#include "swerve_mppi_ros/control_support.hpp"
using namespace swerve_mppi;
namespace chassis = swerve_chassis_core;
int main()
{
  Config config;
  std::vector<std::vector<Pose>> paths{
    {{0, 0, 0}, {1, 0, 0}},
    {{0, 0, 0}, {0, 0.8, 0}},
    {{0, 0, 0}, {-0.8, 0, 0}},
    {{0, 0, 0}, {0, 0, 0.9}},
    {{0, 0, 0}, {0.7, 0.4, 0.6}},
    {{0, 0, 0}, {0.5, 0.1, 0}, {1, -0.1, 0}, {1.5, 0, 0}},
    {{0, 0, 0}, {0.7, 0, 0}, {0, 0, 0}},
    {{0,0,0},{.8,0,0},{.8,.8,0},{0,.8,0},{0,0,0}}};
  std::vector<double> times;
  for (std::size_t scenario = 0; scenario < paths.size(); ++scenario) {
    Controller controller(config);
    chassis::Generator executor(config.geometry, config.limits);
    swerve_mppi_ros::PathHandler path;
    path.set(paths[scenario]);
    swerve_mppi_ros::GoalChecker goal;
    State s;
    s.generator.valid = true;
    bool reached = false;
    for (int step = 0; step < 800; ++step) {
      Input input;
      input.state = s;
      input.path = path.local(s.pose);
      input.goal = path.goal();
      input.goal_active = path.goal_active();
      auto result = controller.compute(input);
      times.push_back(result.compute_ms);
      if (result.status != Status::Ok) throw std::runtime_error("no valid closed-loop command");
      for (int tick = 0; tick < 5; ++tick) {
        auto out = executor.step(
          {result.command.velocity.vx, result.command.velocity.vy, result.command.velocity.wz},
          {s.steering, s.wheel_speeds}, s.generator, 0.01);
        auto v = chassis::forward(config.geometry, {s.steering, s.wheel_speeds});
        auto p = chassis::integrate({s.pose.x, s.pose.y, s.pose.yaw}, v, 0.01);
        s.pose = {p.x, p.y, p.yaw};
        auto response_limits = config.limits;
        response_limits.steering_rate *= 0.8;
        auto joints =
          chassis::response({s.steering, s.wheel_speeds}, out.state.command, response_limits, 0.01);
        s.generator = out.state;
        s.steering = joints.steering;
        s.wheel_speeds = joints.speed;
        v = chassis::forward(config.geometry, joints);
        s.velocity = {v.vx, v.vy, v.wz};
      }
      if (path.goal_active() && goal.reached(s, path.goal(), step * 0.05)) {
        reached = true;
        std::cout << "scenario=" << scenario << " simulated_s=" << step * 0.05
                  << " error=" << distance(s.pose, path.goal()) << '\n';
        break;
      }
    }
    if (!reached) {
      std::cerr << "failed scenario " << scenario << " pose=" << s.pose.x << ',' << s.pose.y << ','
                << s.pose.yaw << '\n';
      return 1;
    }
  }
  std::sort(times.begin(), times.end());
  std::cout << "compute_ms P50=" << times[times.size() / 2]
            << " P95=" << times[times.size() * 95 / 100]
            << " P99=" << times[times.size() * 99 / 100] << '\n';
}
