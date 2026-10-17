#include <iostream>
#include <stdexcept>

#include "swerve_mppi_ros/control_support.hpp"
#include "swerve_mppi_ros/costmap_critic.hpp"
using namespace swerve_mppi_ros;
void check(bool c, const char * m)
{
  if (!c) throw std::runtime_error(m);
}
int main()
{
  try {
    ChassisAdapter adapter;
    ChassisFeedback feedback;
    feedback.request_id = 7;
    adapter.observe(feedback);
    auto request = adapter.translate({{0, 0.3, 0}, Mode::Crab});
    check(
      request.request_id == 8 && request.command.velocity.vy == 0 && request.entry.vy == 0.3,
      "new mode zero request");
    auto retry = adapter.translate({{0.4, 0, 0}, Mode::Ackermann});
    check(
      retry.request_id == 8 && retry.entry.vy == 0.3 && retry.command.mode == Mode::Crab,
      "frozen switch request");
    check(
      adapter.state().switching && adapter.state().requested_mode == Mode::Crab,
      "pending switch is observed by core");
    feedback.request_id = 8;
    feedback.state.mode = feedback.state.requested_mode = Mode::Crab;
    feedback.entry = {0, 0.3, 0};
    adapter.observe(feedback);
    auto drive = adapter.translate({{0, 0.2, 0}, Mode::Crab});
    check(
      drive.command.velocity.vy == 0.2 && drive.request_id == 8 && drive.entry.vy == 0.3,
      "drive retains receipt");
    feedback.fault = true;
    adapter.observe(feedback);
    auto zero = adapter.translate({{}, Mode::Crab});
    check(zero.request_id == 9 && zero.command.velocity.vy == 0, "startup fault zero");
    GoalChecker checker;
    State state;
    Pose goal;
    check(!checker.reached(state, goal, 1), "settle start");
    check(checker.reached(state, goal, 1.31), "settle complete");
    state.velocity.vx = 0.1;
    check(!checker.reached(state, goal, 1.4), "moving is not reached");
    state.velocity = {};
    state.pose.yaw = 0.2;
    check(!checker.reached(state, goal, 1.5), "heading required");
    PathHandler path;
    path.set({{0, 0, 0}, {1, 0, 0}, {1, 1, 1.57}, {0, 1, 3.14}, {0, 0, 0}});
    auto local = path.local({0, 0, 0}, 0.5);
    check(local.size() == 2 && local.back().x == 1, "crossing keeps initial segment");
    CostmapCritic critic;
    std::uint8_t bytes[] = {0, 254, 255, 0};
    Input input;
    input.map = {bytes, 2, 2, 1, 0, 0};
    Config config;
    std::vector<Trajectory> batch(3);
    for (auto & t : batch) t.states.resize(1);
    batch[0].states[0].pose = {0.5, 0.5, 0};
    batch[1].states[0].pose = {1.5, 0.5, 0};
    batch[2].states[0].pose = {0.5, 1.5, 0};
    std::vector<double> costs(3);
    critic.score({input, config}, batch, costs);
    check(
      costs[0] == 0 && !std::isfinite(costs[1]) && !std::isfinite(costs[2]),
      "ROS-injected grid cost");
    std::cout << "ROS support tests passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
