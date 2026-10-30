#include <iostream>
#include <stdexcept>

#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/heading_reference.hpp"
#include "swerve_mppi/ordered_path.hpp"
using namespace swerve_mppi;
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}
class Capture final : public Critic
{
public:
  mutable double weight = -1;
  mutable ReferenceKind kind = ReferenceKind::ExternalPath;
  void score(
    const CriticContext & c, const std::vector<Trajectory> &, std::vector<double> &) const override
  {
    weight = c.input.goal_heading_weight;
    kind = c.input.reference_kind;
  }
};
int main()
{
  try {
    Config config;
    Input input;
    input.state.generator.valid = true;
    input.path = {{0, 0, 0}, {.1, .02, 2}, {.2, .04, -2}, {.7, .2, 1}};
    input.goal = input.path.back();
    auto changed = input;
    changed.path[0].yaw = -3;
    changed.path[1].yaw = 1.5;
    changed.path[2].yaw = 2.8;
    const auto a = Controller(config).compute(input), b = Controller(config).compute(changed);
    require(a.status == Status::Ok && b.status == Status::Ok, "external path computes");
    require(std::abs(a.cost - b.cost) < 1e-9, "intermediate yaw cannot change costs");
    require(a.controls.size() == b.controls.size(), "same control horizon");
    for (std::size_t i = 0; i < a.controls.size(); ++i)
      require(
        std::abs(a.controls[i].vx - b.controls[i].vx) +
            std::abs(a.controls[i].vy - b.controls[i].vy) +
            std::abs(a.controls[i].wz - b.controls[i].wz) <
          1e-10,
        "same controls with changed intermediate yaw");
    OrderedPath route;
    route.set({{0, 0, 1}, {1, 0, -2}, {1, 0, 2}, {1, 1, -.9}});
    require(!route.rotating() && route.goal().yaw == -.9, "goal yaw retained independently");
    const auto local = route.local({0, 0, 0});
    require(std::abs(local.front().yaw) < 1e-9, "forward lookahead does not cross sharp corner");
    route.local({1, 0, 0});
    require(!route.rotating(), "intermediate repeated-position yaw is ignored");
    input.reference_kind = ReferenceKind::GeometricPath;
    input.path = {{0, 0, 0}, {1, 0, 0}};
    input.goal = {1, 0, 1.57};
    input.remaining_arc = .3;
    input.state.pose = {.7, 0, 0};
    const double weight = terminal_heading_weight(input, config);
    require(std::abs(weight - .5) < 1e-9, "measured 30 cm distance gives half weight");
    input.goal_active = false;
    require(terminal_heading_weight(input, config) == 0, "ordered gate blocks coincident endpoint");
    input.goal_active = true;
    input.remaining_arc = 0;
    input.state.pose = {0, 0, 0};
    require(
      terminal_heading_weight(input, config) == 0,
      "off-path distance prevents premature final-only control");
    input.state.pose = {.7, 0, 0};
    input.remaining_arc = .3;
    auto capture = std::make_shared<Capture>();
    auto critics = tracking_critics();
    critics.push_back(capture);
    Controller controller(config, critics);
    auto result = controller.compute(input);
    require(
      result.status == Status::Ok && result.motion_reference.has_value(),
      "normal tracking retains effective reference");
    require(std::abs(capture->weight - weight) < 1e-9, "scoring uses common measured weight");
    input.goal.yaw = -2;  // Retiming must retain the already-selected reference.
    require(controller.retime(input, .025, result), "retiming validates retained reference");
    require(std::abs(capture->weight - weight) < 1e-9, "retiming retains blend weight");
    input.reference_kind = ReferenceKind::FixedHeading;
    require(
      terminal_heading_weight(input, config) == 1, "internal fixed heading has full authority");
    input.reference_kind = ReferenceKind::Departure;
    require(
      terminal_heading_weight(input, config) == 0, "departure is isolated from final goal blend");
    route.set({{0, 0, 0}, {0, 0, 1.2}});
    require(
      route.goal().yaw == 1.2 && route.local({0, 0, 0}).size() == 1,
      "pure rotation remains supported");
    config.goal_heading_full_distance = config.goal_heading_start_distance;
    bool rejected = false;
    try {
      Controller invalid(config);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "invalid blend interval rejected");
    std::cout << "heading reference tests passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
