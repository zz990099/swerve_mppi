// Reproducible numerical recordings; this is not a ROS/Gazebo integration run.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "swerve_mppi/controller.hpp"
#include "swerve_mppi_ros/control_support.hpp"
using namespace swerve_mppi;
namespace chassis = swerve_chassis_core;
int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  const std::filesystem::path root(argv[1]);
  std::ifstream cases(root / "cases.txt");
  std::ofstream metrics(root / "metrics.csv");
  metrics << "scenario,status,duration_s,path_peak_m,final_position_m,final_yaw_rad,no_solution,"
             "compute_p99_ms\n";
  std::string name;
  while (std::getline(cases, name)) {
    std::ifstream file(root / "references" / (name + ".csv"));
    std::vector<Pose> points;
    std::string line;
    while (std::getline(file, line)) {
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      Pose p;
      if (row >> p.x >> p.y >> p.yaw) points.push_back(p);
    }
    if (points.empty()) return 3;
    Config config;
    config.seed = 42;
    Controller controller(config);
    OrderedPath path;
    path.set(points, config.lookahead);
    chassis::Generator executor(config.geometry, config.limits);
    swerve_mppi_ros::GoalChecker checker;
    State state;
    state.generator.valid = true;
    std::ofstream trace(root / (name + ".csv"));
    trace << std::setprecision(12);
    trace << "t,x,y,yaw,vx,vy,wz,request_vx,request_vy,request_wz,remaining,goal_weight,branch_"
             "phase,compute_ms,path_error\n";
    double peak = 0, time = 0, last_motion = 0;
    Pose motion_pose;
    std::string status = "timeout";
    std::size_t failures = 0;
    std::vector<double> times;
    for (int step = 0; step < 1800; ++step) {
      time = step * .05;
      Input input;
      input.reference_kind = ReferenceKind::GeometricPath;
      input.state = state;
      input.path = path.local(state.pose, 3., state.velocity);
      input.goal = path.goal();
      input.goal_active = path.goal_active(config.goal_heading_start_distance);
      input.remaining_arc = path.remaining();
      auto result = controller.compute(input);
      if (result.completed_transition) path.recover(state.pose);
      times.push_back(result.compute_ms);
      if (result.status != Status::Ok) {
        ++failures;
        status = "no_solution";
        break;
      }
      for (int tick = 0; tick < 5; ++tick) {
        auto out = executor.step(
          {result.command.velocity.vx, result.command.velocity.vy, result.command.velocity.wz},
          {state.steering, state.wheel_speeds}, state.generator, .01);
        auto body = chassis::forward(config.geometry, {state.steering, state.wheel_speeds});
        auto p = chassis::integrate({state.pose.x, state.pose.y, state.pose.yaw}, body, .01);
        state.pose = {p.x, p.y, p.yaw};
        auto target = out.state.command;
        // Independent servo plant is 20% slower than the controller model.
        for (std::size_t w = 0; w < 4; ++w)
          target.steering[w] =
            state.steering[w] - std::expm1(-.01 / .12) * (target.steering[w] - state.steering[w]);
        auto joints =
          chassis::response({state.steering, state.wheel_speeds}, target, config.limits, .01);
        state.steering = joints.steering;
        state.wheel_speeds = joints.speed;
        state.generator = out.state;
        body = chassis::forward(config.geometry, joints);
        state.velocity = {body.vx, body.vy, body.wz};
      }
      state.requested = result.command.velocity;
      double error = PathView(points).match(state.pose).error;
      peak = std::max(peak, error);
      trace << time << ',' << state.pose.x << ',' << state.pose.y << ',' << state.pose.yaw << ','
            << state.velocity.vx << ',' << state.velocity.vy << ',' << state.velocity.wz << ','
            << state.requested.vx << ',' << state.requested.vy << ',' << state.requested.wz << ','
            << path.remaining() << ',' << result.goal_heading_weight << ',' << result.branch_phase
            << ',' << result.compute_ms << ',' << error << '\n';
      if (
        path.goal_active() && std::hypot(state.requested.vx, state.requested.vy) < 1e-4 &&
        std::abs(state.requested.wz) < 1e-4 && checker.reached(state, path.goal(), time)) {
        status = peak <= .1 ? "passed" : "path_deviation";
        break;
      }
      if (
        distance(state.pose, motion_pose) > .02 ||
        std::abs(wrap_angle(state.pose.yaw - motion_pose.yaw)) > .05) {
        last_motion = time;
        motion_pose = state.pose;
      }
      if (time - last_motion > 15) {
        status = "stalled";
        break;
      }
      if (peak > .5) {
        status = "large_deviation";
        break;
      }
    }
    std::sort(times.begin(), times.end());
    const double p99 =
      times.empty() ? 0 : times[std::min(times.size() - 1, times.size() * 99 / 100)];
    metrics << name << ',' << status << ',' << time << ',' << peak << ','
            << distance(state.pose, path.goal()) << ','
            << std::abs(wrap_angle(state.pose.yaw - path.goal().yaw)) << ',' << failures << ','
            << p99 << std::endl;
    std::cout << name << " " << status << " " << time << "s peak=" << peak << std::endl;
  }
}
