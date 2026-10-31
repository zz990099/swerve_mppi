#include <iostream>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/heading_reference.hpp"
#include "swerve_mppi/ordered_path.hpp"
#include "swerve_mppi/reference.hpp"
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
  mutable bool hold_candidate = false, turn_candidate = false;
  void score(const CriticContext & c, const std::vector<Trajectory> & batch, std::vector<double> &)
    const override
  {
    weight = c.input.goal_heading_weight;
    kind = c.input.reference_kind;
    for (const auto & t : batch)
      if (t.valid && !t.controls.empty()) {
        hold_candidate |= std::all_of(
          t.controls.begin(), t.controls.end(), [](Velocity u) { return std::abs(u.wz) < 1e-10; });
        turn_candidate |=
          std::any_of(t.controls.begin(), t.controls.end(), [](Velocity u) { return u.wz > .01; });
      }
  }
};
class Reject final : public Critic
{
  void score(const CriticContext &, const std::vector<Trajectory> &, std::vector<double> & costs)
    const override
  {
    for (auto & cost : costs) cost = std::numeric_limits<double>::infinity();
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
    Input startup;
    startup.reference_kind = ReferenceKind::GeometricPath;
    startup.state.generator.valid = true;
    startup.path = {{0, 0, 1.57079632679}, {0, 3, 1.57079632679}};
    startup.goal = startup.path.back();
    startup.goal_active = false;
    startup.goal_heading_weight = 0;
    ReferenceProfile straight(startup, config);
    require(
      std::abs(straight.unit(.2, 0).wz) < 1e-12,
      "ordinary XY speed reference does not impose body yaw rate");
    require(
      straight.heading_preference(0) > .99, "long straight route has full direction preference");
    auto corner = startup;
    corner.path = {{0, 0, 0}, {.5, 0, 0}, {.5, .5, 1.57079632679}, {1, .5, 0}};
    ReferenceProfile complex(corner, config);
    require(
      complex.heading_preference(0) < .2 * straight.heading_preference(0),
      "short repeated turns continuously weaken preference");
    auto arbitrary = startup;
    arbitrary.path[0].yaw = -2;
    arbitrary.path[1].yaw = 2;
    ReferenceProfile no_yaw_coupling(arbitrary, config);
    require(
      std::abs(straight.speed(.2) - no_yaw_coupling.speed(.2)) < 1e-10,
      "ordinary nominal speed is independent of prescribed body yaw changes");
    auto proposals = std::make_shared<Capture>();
    auto proposal_critics = tracking_critics();
    proposal_critics.push_back(proposals);
    const auto choice = Controller(config, proposal_critics).compute(startup);
    require(
      choice.status == Status::Ok && choice.branch_phase == 0 &&
        choice.motion_reference->reference_kind == ReferenceKind::GeometricPath,
      "ordinary startup keeps free yaw and has no mandatory body alignment phase");
    require(
      proposals->hold_candidate && proposals->turn_candidate,
      "physically valid keep-heading and gentle-alignment candidates both reach scoring");
    const auto heading = tracking_critics().back();
    Trajectory aligned, opposite;
    aligned.states = {startup.state, startup.state};
    opposite = aligned;
    aligned.states.back().pose = {0, .05, 1.57079632679};
    opposite.states.back().pose = {0, .05, -1.57079632679};
    aligned.controls = {{0, .1, 0}};
    opposite.controls = aligned.controls;
    std::vector<double> costs(2, 0);
    heading->score({startup, config, &straight}, {aligned, opposite}, costs);
    require(
      std::abs(costs[0]) < 1e-10 && costs[1] > costs[0] && costs[1] <= .5 + 1e-10 &&
        std::isfinite(costs[1]),
      "travel heading is a bounded preference and opposite yaw remains admissible");
    auto finishing = startup;
    finishing.goal_active = true;
    finishing.goal_heading_weight = 1;
    finishing.goal = {0, 3, -1.57079632679};
    costs.assign(2, 0);
    heading->score({finishing, config, &straight}, {aligned, opposite}, costs);
    require(costs[1] < costs[0], "final task yaw is scored independently of geometric heading");
    Input arrived;
    arrived.state.generator.valid = true;
    arrived.state.pose = {.024, 0, .017};
    arrived.path = {{0, 0, 0}};
    arrived.goal = {0, 0, 0};
    arrived.remaining_arc = 0;
    arrived.state.requested = {-.008, -.027, .034};
    const auto held = Controller(config).compute(arrived);
    require(
      held.status == Status::Ok &&
        std::hypot(held.command.velocity.vx, held.command.velocity.vy) < 1e-10 &&
        std::abs(held.command.velocity.wz) < 1e-10,
      "arrived and stopped pose emits validated zero inside shared completion tolerances");
    require(
      Controller(config, {std::make_shared<Reject>()}).compute(arrived).status ==
        Status::NoSolution,
      "terminal zero request cannot bypass a hard critic rejection");
    arrived.goal_tolerance.yaw = .005;
    const auto correcting = Controller(config).compute(arrived);
    require(
      correcting.status == Status::Ok && std::abs(correcting.command.velocity.wz) > 1e-5,
      "custom tighter goal tolerance retains heading correction");
    arrived.path = {{.024, 0, .017}, {1, 0, 0}, {.024, 0, .017}};
    arrived.goal = arrived.path.back();
    arrived.remaining_arc = -1;
    require(
      !terminal_position_reached(arrived),
      "coincident endpoint cannot bypass ordered remaining distance");
    arrived.goal_tolerance.yaw = -1;
    require(
      Controller(config).compute(arrived).status == Status::InvalidInput,
      "invalid completion tolerances rejected");
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
