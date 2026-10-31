#include <iostream>
#include <limits>
#include <stdexcept>

#include "data/stopped_states.hpp"
#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/ordered_path.hpp"
#include "swerve_mppi/reference.hpp"
using namespace swerve_mppi;
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}
class Reject final : public Critic
{
  void score(const CriticContext &, const std::vector<Trajectory> &, std::vector<double> & costs)
    const override
  {
    for (auto & c : costs) c = std::numeric_limits<double>::infinity();
  }
};
int main()
{
  try {
    Config config;
    config.max_speed = config.limits.linear_speed = 1;
    for (auto input : {test_data::figure8_stopped(), test_data::reverse_stopped()}) {
      input.record_decisions = true;
      const auto result = Controller(config).compute(input);
      require(result.status == Status::Ok, "captured stopped state has a solution");
      require(
        result.decisions.size() == config.samples + 1, "complete candidate and weighted audit");
      const DecisionDiagnostic * selected = nullptr;
      const DecisionDiagnostic * brake = nullptr;
      for (const auto & d : result.decisions) {
        if (d.selected) selected = &d;
        if (d.candidate == 2) brake = &d;
      }
      require(
        selected && selected->valid && brake && brake->valid,
        "selected and brake decisions recorded");
      require(
        selected->cost < brake->cost, "useful feasible motion beats parking at the captured state");
      require(
        std::hypot(result.command.velocity.vx, result.command.velocity.vy) +
            .3 * std::abs(result.command.velocity.wz) >
          1e-5,
        "captured unfinished task does not select zero request");
      require(
        selected->tracking.alignment == 0,
        "ordinary steering work is not double charged as initial setup");
      auto unrecorded = input;
      unrecorded.record_decisions = false;
      const auto same = Controller(config).compute(unrecorded);
      require(
        same.decisions.empty() && std::abs(same.cost - result.cost) < 1e-10 &&
          std::abs(same.command.velocity.vx - result.command.velocity.vx) +
              std::abs(same.command.velocity.vy - result.command.velocity.vy) +
              std::abs(same.command.velocity.wz - result.command.velocity.wz) <
            1e-10,
        "optional diagnostics do not alter optimization");
      require(
        Controller(config, {std::make_shared<Reject>()}).compute(input).status ==
          Status::NoSolution,
        "liveness proposals cannot bypass a hard rejection");
      if (!input.goal_active)
        require(selected->predicted_progress > .03, "route restart predicts ordered progress");
    }
    Input moving;
    moving.reference_kind = ReferenceKind::GeometricPath;
    moving.state.generator.valid = true;
    moving.state.velocity = {.2, 0, 0};
    moving.state.requested = {.2, 0, 0};
    moving.state.wheel_speeds.fill(.2 / config.geometry.radius);
    moving.state.generator.command.speed = moving.state.wheel_speeds;
    moving.path = {{0, 0, 0}, {.4, 0, 0}};
    moving.goal = {.4, 0, 3.141592653589793};
    moving.remaining_arc = .4;
    moving.record_decisions = true;
    const auto stopping = Controller(config).compute(moving);
    require(stopping.status == Status::Ok, "moving terminal state has feasible proposals");
    bool stop_then_turn = false;
    for (const auto & d : stopping.decisions)
      if (d.candidate == 7 && d.valid)
        stop_then_turn = std::abs(d.conditioned_first.wz) < 1e-10 &&
                         d.conditioned_first.vx < moving.state.requested.vx && d.yaw_motion > .1 &&
                         d.xy_motion < .1;
    require(stop_then_turn, "terminal rotation proposal predicts braking before turning");
    OrderedPath straight;
    std::vector<Pose> dense;
    for (int i = 0; i <= 240; ++i) dense.push_back({i * .05, 0, 0});
    straight.set(dense);
    Input cached;
    cached.reference_kind = ReferenceKind::GeometricPath;
    cached.path = straight.local({0, 0, 0});
    cached.heading_preferences = straight.heading_preferences();
    require(
      valid_heading_preferences(cached), "cached preferences stay within closed unit interval");
    std::vector<Pose> route;
    for (int i = 0; i <= 40; ++i) route.push_back({i * .1, 0, 0});
    for (int i = 1; i <= 40; ++i) route.push_back({4, i * .1, 0});
    OrderedPath ordered;
    ordered.set(route);
    for (int i = 0; i <= 30; ++i) ordered.local({i * .05, 0, 0});
    Input short_crop;
    short_crop.reference_kind = ReferenceKind::GeometricPath;
    short_crop.state.pose = {1.5, 0, 0};
    short_crop.path = ordered.local(short_crop.state.pose, 1.);
    short_crop.heading_preferences = ordered.heading_preferences();
    short_crop.goal_active = false;
    auto long_crop = short_crop;
    long_crop.path = ordered.local(short_crop.state.pose, 3.);
    long_crop.heading_preferences = ordered.heading_preferences();
    ReferenceProfile short_reference(short_crop, config), long_reference(long_crop, config);
    require(
      std::abs(short_reference.heading_preference(.5) - long_reference.heading_preference(.5)) <
        1e-9,
      "same global progress has identical preference across different local crop lengths");
    auto snapshot = test_data::figure8_stopped();
    snapshot.heading_preferences.assign(snapshot.path.size(), .25);
    Controller controller(config);
    auto captured = controller.compute(snapshot);
    require(
      captured.status == Status::Ok &&
        captured.motion_reference->heading_preferences == snapshot.heading_preferences,
      "result retains complete route context");
    snapshot.heading_preferences = {-1};
    require(
      controller.retime(snapshot, .025, captured),
      "retime uses retained route context rather than changed caller metadata");
    require(
      Controller(config).compute(snapshot).status == Status::InvalidInput,
      "invalid route context rejected");
    std::cout << "liveness snapshot contracts passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
