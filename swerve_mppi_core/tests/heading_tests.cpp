#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/critic.hpp"
#include "swerve_mppi/reference.hpp"
using namespace swerve_mppi;
void check(bool ok, const char * message)
{
  if (!ok) throw std::runtime_error(message);
}
int main()
{
  try {
    Config config;
    ReferenceProfile line({{0, 0, 2}, {5, 0, -2}}, config);
    std::vector<Pose> dense;
    for (int i = 0; i <= 100; ++i) dense.push_back({.05 * i, 0, std::sin(i)});
    ReferenceProfile resampled(dense, config);
    const auto straight = line.heading(2.);
    check(
      std::abs(straight.yaw) < 1e-12 && straight.strength > .99,
      "long straight has a strong geometric preference independent of pose yaw");
    check(
      std::abs(straight.strength - resampled.heading(2).strength) < 1e-10 &&
        std::abs(straight.yaw - resampled.heading(2).yaw) < 1e-10,
      "waypoint density and arbitrary intermediate yaw do not alter the preference");
    ReferenceProfile duplicates({{0, 0, 0}, {0, 0, 1}, {5, 0, 0}}, config);
    check(duplicates.heading(2).strength > .99, "coincident positions are harmless");
    ReferenceProfile fold({{0, 0, 0}, {1, 0, 0}, {0, 0, 0}}, config);
    check(fold.heading(1).strength < .05, "an ambiguous cusp releases body orientation");
    ReferenceProfile turns({{0, 0, 0}, {.3, .3, 0}, {.6, -.3, 0}, {.9, 0, 0}}, config);
    check(turns.heading(.7).strength < .1, "opposing sharp turns must not cancel complexity");
    ReferenceProfile crossing({{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {0, -2, 0}}, config);
    check(
      std::abs(crossing.heading(.1).yaw) < .01,
      "a later spatial crossing cannot change this ordered arc direction");
    ReferenceProfile spin({{0, 0, 1}}, config);
    check(spin.heading(0).strength == 0, "pure rotation has no geometric direction");
    check(
      line.heading(5, true).strength == 0 && line.heading(4.5, true).strength > 0 &&
        line.heading(4.5, true).strength < line.heading(4., true).strength,
      "geometric preference fades continuously before terminal yaw takes over");
    Input input;
    input.path = {{0, 0, 0}, {5, 0, 0}};
    input.goal_active = false;
    std::vector<Trajectory> batch(2);
    for (int k = 0; k < 2; ++k) {
      batch[k].states.resize(2);
      batch[k].states[1].pose = {.1, 0, k * std::acos(-1.) / 2};
      batch[k].controls.resize(1);
    }
    std::vector<TrackingCost> detail;
    CriticContext context{input, config, nullptr, &detail};
    std::vector<double> costs(2, 0);
    tracking_critics().front()->score(context, batch, costs);
    check(
      std::isfinite(costs[1]) && costs[0] < costs[1] && detail[1].heading > 0,
      "body alignment is preferred but a misaligned or stopped candidate stays finite");
    input.path = {{0, 0, 0}, {.51, 0, 1}};
    input.goal = input.path.back();
    input.preview_reaches_goal = true;
    costs.assign(2, 0);
    tracking_critics().front()->score(context, batch, costs);
    const double before_goal_activation = detail[1].heading;
    input.goal_active = true;
    costs.assign(2, 0);
    tracking_critics().front()->score(context, batch, costs);
    check(
      before_goal_activation == detail[1].heading && before_goal_activation > 0,
      "geometric fade is independent of terminal attraction activation");
    input.preview_reaches_goal = false;
    costs.assign(2, 0);
    tracking_critics().front()->score(context, batch, costs);
    check(
      detail[1].heading > before_goal_activation, "a crop does not fade at an apparent endpoint");
    const double pi = swerve_chassis_core::pi;
    check(
      std::abs(match_geometric_heading(pi, 0., config).error) > 3,
      "default heading policy prefers turning forward from reverse");
    config.geometric_heading_allow_reverse = true;
    check(
      std::abs(match_geometric_heading(pi, 0., config).error) < 1e-9 &&
        match_geometric_heading(pi / 2, 0., config).error < 0 &&
        match_geometric_heading(-pi / 2, 0., config).error > 0,
      "reverse alignment is accepted and both perpendicular ties prefer forward");
    config.geometric_heading_weight = 0;
    costs.assign(2, 0);
    tracking_critics().front()->score(context, batch, costs);
    check(
      detail[0].heading == 0 && detail[1].heading == 0,
      "zero weight removes the geometric heading objective");
    config.geometric_heading_window = std::numeric_limits<double>::quiet_NaN();
    bool rejected = false;
    try {
      validate(config);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected, "invalid geometric preference parameters are rejected");
    std::cout << "geometric heading contracts passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
