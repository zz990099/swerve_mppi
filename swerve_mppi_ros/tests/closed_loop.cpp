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
    {{0, 0, 0}, {.8, 0, 0}, {.8, .8, 0}, {0, .8, 0}, {0, 0, 0}},
    {{0, 0, 0}, {.4, .15, 0}, {.8, -.15, 0}, {1.2, 0, 0}}};
  paths.push_back({{0, 0, 0}, {0, -.8, 0}});
  std::vector<Pose> smooth_s, bezier, reverse_curve;
  for (int i = 0; i <= 40; ++i) {
    double t = i / 40.;
    smooth_s.push_back({1.8 * t, .22 * std::sin(2 * chassis::pi * t), 0});
    double x = 3 * (1 - t) * (1 - t) * t * .5 + 3 * (1 - t) * t * t * 1.1 + t * t * t * 1.6;
    double y = 3 * (1 - t) * t * t * .6 + t * t * t * .6;
    double dx = 3 * ((1 - t) * (1 - t) * .5 + 2 * (1 - t) * t * .6 + t * t * .5),
           dy = 3 * 2 * (1 - t) * t * .6;
    double yaw = std::atan2(dy, dx);
    bezier.push_back({x, y, yaw});
    reverse_curve.push_back({-x, -y, yaw});
  }
  paths.push_back(smooth_s);
  paths.push_back(bezier);
  paths.push_back(reverse_curve);
  paths.push_back({{0, 0, 0}, {.8, 0, -.9}});
  std::vector<double> times;
  for (int plant = 0; plant < 2; ++plant) {
    for (std::size_t scenario = 0; scenario < paths.size(); ++scenario) {
      Controller controller(config);
      chassis::Generator executor(config.geometry, config.limits);
      swerve_mppi_ros::PathHandler path;
      path.set(paths[scenario]);
      swerve_mppi_ros::GoalChecker goal;
      State s;
      s.generator.valid = true;
      bool reached = false;
      auto geometry = geometric_path(paths[scenario], config.lookahead);
      PathView reference(geometry);
      double square_error = 0, peak = 0, heading_square = 0, heading_peak = 0, settled_since = -1;
      int observations = 0, heading_observations = 0, stopped = 0;
      for (int step = 0; step < 800; ++step) {
        Input input;
        input.reference_kind = ReferenceKind::GeometricPath;
        input.state = s;
        const auto previous_section = path.section();
        input.path = path.local(s.pose, 3.0, s.velocity);
        if (path.section() != previous_section) controller.reset();
        input.goal = path.tracking_goal();
        input.goal_active = path.goal_active(config.goal_heading_start_distance);
        input.remaining_arc = path.remaining();
        auto result = controller.compute(input);
        if (result.completed_transition) path.recover(s.pose);
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
          auto target = out.state.command;
          {
            // Independent nominal and 20% slower servo responses.
            const double gain = 1 - std::exp(-0.01 / (plant == 0 ? .10 : .12));
            for (std::size_t i = 0; i < 4; ++i)
              target.steering[i] = s.steering[i] + gain * (target.steering[i] - s.steering[i]);
          }
          auto joints =
            chassis::response({s.steering, s.wheel_speeds}, target, response_limits, 0.01);
          s.generator = out.state;
          s.steering = joints.steering;
          s.wheel_speeds = joints.speed;
          v = chassis::forward(config.geometry, joints);
          s.velocity = {v.vx, v.vy, v.wz};
        }
        double dv = std::hypot(
          result.command.velocity.vx - s.requested.vx, result.command.velocity.vy - s.requested.vy);
        double dw = std::abs(result.command.velocity.wz - s.requested.wz);
        if (
          dv > config.limits.linear_acceleration * .05 + 1e-7 ||
          dw > config.limits.angular_acceleration * .05 + 1e-7)
          throw std::runtime_error("request slew exceeded");
        s.requested = result.command.velocity;
        auto match = reference.match(s.pose);
        double angle = std::abs(wrap_angle(s.pose.yaw - match.pose.yaw));
        square_error += match.error * match.error;
        peak = std::max(peak, match.error);
        if (
          result.goal_heading_weight < .01 && result.branch_phase == 0 &&
          std::hypot(s.velocity.vx, s.velocity.vy) > .1) {
          heading_square += angle * angle;
          heading_peak = std::max(heading_peak, angle);
          ++heading_observations;
        }
        ++observations;
        double wheel_peak = 0;
        for (double v : s.wheel_speeds) wheel_peak = std::max(wheel_peak, std::abs(v));
        stopped += wheel_peak < .05;
        if (wheel_peak >= .05)
          settled_since = -1;
        else if (settled_since < 0)
          settled_since = step * .05;
        if (
          path.goal_active() && goal.reached(s, path.goal(), step * 0.05) && settled_since >= 0 &&
          step * .05 - settled_since >= .3) {
          reached = true;
          std::cout << "plant=" << plant << " scenario=" << scenario
                    << " simulated_s=" << step * 0.05 << " error=" << distance(s.pose, path.goal())
                    << " rms=" << std::sqrt(square_error / observations) << " peak=" << peak
                    << " stopped=" << double(stopped) / observations << " heading_rms="
                    << std::sqrt(heading_square / std::max(1, heading_observations))
                    << " heading_peak=" << heading_peak << std::endl;
          if (
            std::sqrt(square_error / observations) > .03 || peak > .075 ||
            double(stopped) / observations > .55 ||
            (reference.length() > 1e-6 &&
             (std::sqrt(heading_square / std::max(1, heading_observations)) > .05 ||
              heading_peak > .12)))
            throw std::runtime_error("closed-loop tracking quality gate failed");
          break;
        }
      }
      if (!reached) {
        std::cerr << "failed scenario " << scenario << " pose=" << s.pose.x << ',' << s.pose.y
                  << ',' << s.pose.yaw << '\n';
        return 1;
      }
    }
  }
  std::sort(times.begin(), times.end());
  std::cout << "compute_ms P50=" << times[times.size() / 2]
            << " P95=" << times[times.size() * 95 / 100]
            << " P99=" << times[times.size() * 99 / 100] << '\n';
}
