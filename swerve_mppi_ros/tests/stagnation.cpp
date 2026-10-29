#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/ordered_path.hpp"
#include "swerve_mppi_ros/control_support.hpp"
using namespace swerve_mppi;
namespace chassis = swerve_chassis_core;

std::vector<Pose> scaled(std::vector<Pose> points)
{
  const double scale = std::max(1., 12. / PathView(points).length());
  for (auto & point : points) {
    point.x *= scale;
    point.y *= scale;
  }
  std::vector<Pose> out{points.front()};
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto a = points[i - 1], b = points[i];
    int count = std::max(1, static_cast<int>(std::ceil(distance(a, b) / .05)));
    for (int j = 1; j <= count; ++j) {
      double u = double(j) / count;
      out.push_back(
        {a.x + u * (b.x - a.x), a.y + u * (b.y - a.y),
         wrap_angle(a.yaw + u * wrap_angle(b.yaw - a.yaw))});
    }
  }
  return out;
}

int main()
{
  Config config;
  config.max_speed = config.limits.linear_speed = 1.;
  std::vector<Pose> heading, spiral;
  for (int i = 0; i <= 80; ++i) heading.push_back({2. * i / 80, 0, chassis::pi * i / 80});
  for (int i = 0; i <= 240; ++i) {
    double t = 4 * chassis::pi * i / 240;
    spiral.push_back({(.2 + .06 * t) * std::cos(t) - .2, (.2 + .06 * t) * std::sin(t), 0});
  }
  std::vector<std::vector<Pose>> cases{
    scaled(heading), scaled(spiral),
    scaled(
      {{0, 0, 0},
       {.7, 0, 0},
       {.7, 0, chassis::pi / 2},
       {.7, .7, chassis::pi / 2},
       {.7, .7, chassis::pi},
       {0, .7, chassis::pi},
       {0, .7, 3 * chassis::pi / 2},
       {0, 0, 3 * chassis::pi / 2}})};
  for (std::size_t name = 0; name < cases.size(); ++name) {
    OrderedPath route;
    route.set(cases[name]);
    Controller controller(config);
    chassis::Generator executor(config.geometry, config.limits);
    swerve_mppi_ros::GoalChecker goal;
    State state;
    state.generator.valid = true;
    double peak = 0;
    bool reached = false;
    for (int step = 0; step < 1300; ++step) {
      Input input;
      input.state = state;
      const auto before = route.section();
      input.path = route.local(state.pose, 3., state.velocity);
      if (route.section() != before) controller.reset();
      input.goal = route.tracking_goal();
      input.goal_active = route.tracking_goal_active();
      auto result = controller.compute(input);
      if (result.status != Status::Ok)
        throw std::runtime_error("stagnation regression: no solution");
      for (int tick = 0; tick < 5; ++tick) {
        auto out = executor.step(
          {result.command.velocity.vx, result.command.velocity.vy, result.command.velocity.wz},
          {state.steering, state.wheel_speeds}, state.generator, .01);
        auto body = chassis::forward(config.geometry, {state.steering, state.wheel_speeds});
        auto pose = chassis::integrate({state.pose.x, state.pose.y, state.pose.yaw}, body, .01);
        state.pose = {pose.x, pose.y, pose.yaw};
        auto target = out.state.command;
        for (std::size_t w = 0; w < 4; ++w)
          target.steering[w] =
            state.steering[w] - std::expm1(-.01 / .10) * (target.steering[w] - state.steering[w]);
        auto joints =
          chassis::response({state.steering, state.wheel_speeds}, target, config.limits, .01);
        state.generator = out.state;
        state.steering = joints.steering;
        state.wheel_speeds = joints.speed;
        body = chassis::forward(config.geometry, joints);
        state.velocity = {body.vx, body.vy, body.wz};
      }
      state.requested = result.command.velocity;
      peak = std::max(peak, PathView(cases[name]).match(state.pose).error);
      if (peak > .1) throw std::runtime_error("stagnation regression: corridor exceeded");
      if (route.goal_active() && goal.reached(state, route.goal(), step * .05)) {
        reached = true;
        std::cout << "stagnation case=" << name << " seconds=" << step * .05 << " peak=" << peak
                  << std::endl;
        break;
      }
    }
    if (!reached) {
      std::cerr << "stalled case=" << name << " pose=" << state.pose.x << ',' << state.pose.y << ','
                << state.pose.yaw << std::endl;
      return 1;
    }
  }
}
