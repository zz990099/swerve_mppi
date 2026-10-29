#include <iostream>
#include <stdexcept>

#include "swerve_mppi/controller.hpp"
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
    path.set({{0, 0, 0}, {.8, 0, 0}, {.8, .8, 0}, {0, .8, 0}, {0, 0, 0}});
    auto local = path.local({0, 0, 0});
    check(local.size() > 2, "sharp corners remain in the preview");
    check(!path.goal_active(), "closed path cannot finish at the start");
    path.local({.8, 0, 0});
    path.local({.8, .8, 0});
    check(!path.goal_active(), "ordered square still has two legs remaining");
    path.local({0, .8, 0});
    check(!path.goal_active(), "final leg activation uses remaining arc length");
    path.local({0, .4, 0});
    check(path.goal_active(), "final approach activates by distance");
    path.set({{0, 0, 0}, {1, 0, 0}, {0, 0, 0}});
    path.local({0, 0, 0});
    check(!path.goal_active(), "return endpoint cannot attract at startup");
    path.local({1, 0, 0});
    path.local({.4, 0, 0});
    check(path.goal_active(), "return leg reaches final approach");
    std::vector<Pose> dense;
    for (int i = 0; i <= 40; ++i) dense.push_back({0, .8 * i / 40, 0});
    PathHandler sparse, sampled;
    sparse.set({{0, 0, 0}, {0, .8, 0}});
    sampled.set(dense);
    for (double y : {0., .315, .5, .7}) {
      auto a = sparse.local({0, y, 0}), b = sampled.local({0, y, 0});
      check(
        distance(a.front(), b.front()) < 1e-9,
        "sampling does not move local origin ahead of robot");
      check(
        std::abs(sparse.remaining() - sampled.remaining()) < 1e-9,
        "arc progress is sampling invariant");
      check(sparse.goal_active() == sampled.goal_active(), "goal activation is sampling invariant");
    }
    path.set({{0, 0, 1.2}});
    check(path.goal_active(), "rotation remains terminal");
    path.set({{0, 0, 0}, {1, 0, 0}, {1, 0, 1.57}, {1, 1, 1.57}});
    auto incoming = path.local({.99, 0, 0}, 3., {.1, 0, 0});
    check(
      !path.rotating() && incoming.back().yaw == 0,
      "incoming translation must stop before rotation");
    auto rotation = path.local({1, 0, 0});
    check(
      path.rotating() && rotation.back().yaw == 1.57 && !path.goal_active(),
      "intermediate rotation blocks later leg and completion");
    path.local({1, 0, 1.57}, 3., {0, 0, .1});
    check(path.rotating(), "rotation completion requires measured stopping");
    auto outgoing = path.local({1, 0, 1.57});
    check(
      !path.rotating() && outgoing.back().y == 1,
      "aligned stopped rotation releases outgoing translation");
    path.set({{0, 0, 3.12}, {0, 0, -3.12}, {0, 0, -3.12}, {1, 0, -3.12}});
    path.local({0, 0, 3.12});
    check(path.rotating(), "rotation crossing pi remains an event");
    path.local({0, 0, -3.12});
    check(!path.rotating(), "wrapped yaw and duplicate points advance in order");
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
    cells.assign(400, 0);
    costs = {0};
    footprint.score({input, config}, {swept}, costs);
    check(costs[0] == 0, "empty map fast path remains collision free");
    swept.states[0].pose = {.95, 0, 0};
    costs = {0};
    footprint.score({input, config}, {swept}, costs);
    check(!std::isfinite(costs[0]), "empty map fast path preserves footprint bounds");
    swept.states[0].pose = {0, 0, 0};
    cells[10 * 20 + 12] = 254;
    costs = {0};
    footprint.score({input, config}, {swept}, costs);
    check(!std::isfinite(costs[0]), "new obstacle immediately disables empty map fast path");
    input.state.generator.valid = true;
    input.path = {{0, 0, 0}, {.5, 0, 0}};
    input.goal = input.path.back();
    cells[10 * 20 + 10] = 254;
    auto blocked =
      swerve_mppi::Controller(config, {std::make_shared<CostmapCritic>()}).compute(input);
    check(
      blocked.status == Status::NoSolution && blocked.diagnostics.collision_rejected > 0,
      "validated brake cannot override collision rejection");
    std::cout << "ROS support tests passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
