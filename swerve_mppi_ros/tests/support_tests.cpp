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
    check(!path.goal_active(), "closed path cannot finish at its initial pose");
    path.local({1, 0, 0});
    check(!path.goal_active(), "intermediate segment cannot activate terminal goal");
    path.local({1, 1, 0});
    check(!path.goal_active(), "penultimate corner still precedes final segment");
    path.local({0, 1, 0});
    check(path.goal_active(), "final ordered segment activates goal convergence");
    path.set({{0, 0, 0}, {1, 0, 0}, {0, 0, 0}});
    local = path.local({0, 0, 0});
    check(
      local.size() == 2 && local.back().x == 1,
      "local scoring excludes the overlapping return leg");
    check(!path.goal_active(), "out-and-back requires the turnaround first");
    local = path.local({1, 0.5, 0});
    check(
      !path.goal_active() && local.front().x == 0, "off-path projection cannot skip a turnaround");
    path.local({1, 0, 0});
    check(path.goal_active(), "turnaround unlocks return goal");
    for (double angle : {-2.7, -0.3, 0.00001, 1.2}) {
      auto rotate = [angle](double x, double y) {
        return Pose{
          x * std::cos(angle) - y * std::sin(angle), x * std::sin(angle) + y * std::cos(angle),
          angle};
      };
      path.set({rotate(0, 0), rotate(.8, 0), rotate(.8, .8), rotate(0, .8), rotate(0, 0)});
      local = path.local(rotate(.8, .8));
      // Advance in order rather than jumping to a later square vertex.
      path.local(rotate(.8, 0));
      local = path.local(rotate(.8, .8));
      check(
        local.size() == 2 && distance(local.back(), rotate(0, .8)) < 1e-9,
        "rotated square cannot expose its final leg before the last corner");
      check(!path.goal_active(), "rotated corner remains ordered");
    }
    path.set({{0, 0, 0}, {.4, .15, 0}, {.8, -.15, 0}, {1.2, 0, 0}});
    local = path.local({0, 0, 0});
    check(local.size() == 2, "S bend keeps the next sharp corner inside the local section");
    path.set({{0, 0, 1.2}});
    check(path.goal_active(), "single-pose rotation remains a terminal task");
    CostmapCritic critic;
    std::uint8_t bytes[] = {0, 254, 255, 253};
    Input input;
    input.map = {bytes, 2, 2, 1, 0, 0};
    Config config;
    std::vector<Trajectory> batch(4);
    for (auto & t : batch) t.states.resize(1);
    batch[0].states[0].pose = {0.5, 0.5, 0};
    batch[1].states[0].pose = {1.5, 0.5, 0};
    batch[2].states[0].pose = {0.5, 1.5, 0};
    batch[3].states[0].pose = {1.5, 1.5, 0};
    std::vector<double> costs(4);
    critic.score({input, config}, batch, costs);
    check(
      costs[0] == 0 && !std::isfinite(costs[1]) && !std::isfinite(costs[2]) &&
        !std::isfinite(costs[3]),
      "ROS-injected grid cost");
    std::vector<std::uint8_t> cells(400, 0);
    cells[10 * 20 + 12] = 254;
    input.map = {cells.data(), 20, 20, 0.1, -1, -1};
    Trajectory swept;
    swept.states.resize(2);
    swept.states[0].pose = {0, 0, 0};
    swept.states[1].pose = {0.6, 0, 0};
    costs = {0};
    critic.score({input, config}, {swept}, costs);
    check(!std::isfinite(costs[0]), "collision between rollout endpoints");
    swept.states.resize(1);
    swept.states[0].pose = {0, 0, 0};
    CostmapCritic footprint(5, true, 0.3);
    costs = {0};
    footprint.score({input, config}, {swept}, costs);
    check(!std::isfinite(costs[0]), "footprint edge collision away from center");
    std::cout << "ROS support tests passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
