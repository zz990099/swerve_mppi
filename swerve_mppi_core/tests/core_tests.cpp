#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/reference.hpp"
using namespace swerve_mppi;
void check(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
class Reject final : public Critic
{
public:
  void score(const CriticContext &, const std::vector<Trajectory> &, std::vector<double> & costs)
    const override
  {
    for (auto & c : costs) c = std::numeric_limits<double>::infinity();
  }
};
class BrakeOnly final : public Critic
{
public:
  void score(
    const CriticContext &, const std::vector<Trajectory> & batch,
    std::vector<double> & costs) const override
  {
    for (std::size_t i = 0; i < batch.size(); ++i) {
      const auto u = batch[i].controls.back();
      const auto first = batch[i].controls.front();
      if (
        std::hypot(u.vx, u.vy) + std::abs(u.wz) > 1e-9 ||
        std::abs(first.vx - .255) + std::abs(first.vy) + std::abs(first.wz) > 1e-9)
        costs[i] = std::numeric_limits<double>::infinity();
    }
  }
};
class RejectRetiming final : public Critic
{
public:
  void score(
    const CriticContext & context, const std::vector<Trajectory> &,
    std::vector<double> & costs) const override
  {
    if (context.input.dt < .04)
      for (auto & cost : costs) cost = std::numeric_limits<double>::infinity();
  }
};
class ThrowingCritic final : public Critic
{
public:
  void score(
    const CriticContext &, const std::vector<Trajectory> &, std::vector<double> &) const override
  {
    throw std::runtime_error("injected critic failure");
  }
};
int main()
{
  try {
    Config c;
    Model model(c);
    State state;
    state.generator.valid = true;
    std::vector<Pose> coupled{{0, 0, 0}, {1, 0, .8}};
    Config slower = c;
    slower.steering_time_constant *= 2;
    ReferenceProfile profile(coupled, c), slow_profile(coupled, slower);
    ReferenceProfile owned(std::vector<Pose>{{0, 0, 0}, {1, 0, 0}}, c);
    check(owned.speed(.3) > 0, "profile owns temporary path storage");
    check(
      slow_profile.speed(.3) <= profile.speed(.3) && profile.speed(1) == 0,
      "reference slows for servo lag and stops at the endpoint");
    std::vector<Pose> lateral_path{{0, 0, 0}, {0, .8, 0}};
    ReferenceProfile lateral_profile(lateral_path, c);
    auto direction = lateral_profile.unit(.2, 0);
    check(
      std::abs(direction.vx) < 1e-9 && direction.vy > .99 && direction.wz == 0,
      "reference preserves body heading during lateral travel");
    std::vector<Pose> reverse_path{{0, 0, 0}, {-.8, 0, 0}};
    ReferenceProfile reverse_profile(reverse_path, c);
    check(
      reverse_profile.unit(.2, 0).vx < -.99,
      "reverse reference does not rotate body heading to path tangent");
    std::vector<Pose> crossing{{0, 0, 0}, {.1, .1, 0}, {.2, 0, 0}, {.1, -.1, 0}, {.001, 0, 0}};
    check(
      PathView(crossing).start_match({.001, 0, 0}).arc < .01,
      "a closer future crossing cannot initialize progress at the end of the preview");
    std::vector<Pose> ring;
    for (int i = 0; i <= 80; ++i) {
      const double a = 2 * swerve_chassis_core::pi * i / 80;
      ring.push_back({.5 * std::sin(a), .5 * (1 - std::cos(a)), 0});
    }
    check(
      PathView(ring).start_match({-.001, 0, 0}).arc < .02,
      "smooth return leg cannot capture noisy initial matching");
    Input ring_input;
    ring_input.state.generator.valid = true;
    ring_input.state.pose.x = -.001;
    ring_input.path = ring;
    ring_input.goal = ring.back();
    ring_input.goal_active = false;
    const auto ring_result = Controller(c).compute(ring_input);
    check(
      ring_result.status == Status::Ok && ring_result.reference_speed > .05,
      "self-crossing preview cannot create a false terminal speed at startup");
    std::vector<Pose> return_path{{0, 0, 0}, {.7, 0, 0}, {0, 0, 0}};
    check(
      PathView(return_path).match_ordered({.1, 0, 0}, .67, 1.4).arc < .7,
      "prediction cannot skip an unvisited return checkpoint");
    std::vector<Pose> corner_path{{0, 0, 0}, {.8, 0, 0}, {.8, .8, 0}};
    ReferenceProfile corner_profile(corner_path, c);
    check(
      corner_profile.unit(.79, 0).vx > .99 && corner_profile.unit(.8, 0).vy > .99,
      "mandatory corner does not smear steering into the incoming segment");
    auto t = model.rollout(state, std::vector<Velocity>(30, {0, 0.4, 0}));
    check(t.valid && t.states.back().pose.y > 0.3, "lateral presteering rollout");
    auto combo = model.rollout(state, std::vector<Velocity>(c.horizon, {0.3, 0.15, 0.2}));
    check(
      combo.valid && combo.states.back().pose.x > 0.2 && combo.states.back().pose.y > 0.1 &&
        combo.states.back().pose.yaw > 0.2,
      "combined rollout");
    swerve_chassis_core::Generator generator(c.geometry, c.limits);
    Config step = c;
    step.model_dt = step.integration_dt;
    Model one(step);
    auto actual = generator.step(
      {0.3, 0.15, 0.2}, {state.steering, state.wheel_speeds}, state.generator, step.integration_dt);
    auto prediction = one.rollout(state, {{0.3, 0.15, 0.2}});
    check(
      prediction.states.back().generator.command.steering == actual.state.command.steering,
      "shared next-step target");
    check(
      prediction.states.back().steering[0] > 0 &&
        prediction.states.back().steering[0] < actual.state.command.steering[0],
      "measured steering lags the shared target");
    std::array<double, 4> lateral_reference;
    lateral_reference.fill(swerve_chassis_core::pi / 2);
    auto stable = model.condition({-.001, .6, .001}, {0, .6, 0}, lateral_reference, .05);
    auto stable_wheels = swerve_chassis_core::inverse(
      c.geometry, {stable.vx, stable.vy, stable.wz}, lateral_reference);
    for (double angle : stable_wheels.steering)
      check(
        std::abs(angle - swerve_chassis_core::pi / 2) < .01,
        "small lateral correction cannot flip steering by pi");
    auto bounded = model.condition({.8, .8, .8}, {}, state.steering, .05);
    check(
      std::hypot(bounded.vx, bounded.vy) <= c.limits.linear_acceleration * .05 + 1e-9 &&
        std::abs(bounded.wz) <= c.limits.angular_acceleration * .05 + 1e-9,
      "request slew follows accepted previous command");
    state.generator.valid = false;
    check(!model.rollout(state, {{0.2, 0, 0}}).valid, "missing snapshot rejected");
    state.generator.valid = true;
    Input input;
    input.state = state;
    input.path = {{0, 0, 0}, {1, 0, 0}};
    input.goal = input.path.back();
    Controller controller(c);
    auto result = controller.compute(input);
    check(
      result.status == Status::Ok && result.command.velocity.vx > 0, "valid continuous command");
    std::cout << "initial compute ms=" << result.compute_ms << '\n';
    input.state.pose.x = std::numeric_limits<double>::quiet_NaN();
    check(controller.compute(input).status == Status::InvalidInput, "invalid input");
    input.state = state;
    check(controller.compute(input).status == Status::Ok, "input recovery");
    Input lateral;
    lateral.state.generator.valid = true;
    lateral.path = {{0, 0, 0}, {0, .8, 0}};
    lateral.goal = lateral.path.back();
    Controller lateral_controller(c);
    auto lateral_result = lateral_controller.compute(lateral);
    check(
      lateral_result.status == Status::Ok && std::abs(lateral_result.command.velocity.vx) < 1e-6 &&
        std::abs(lateral_result.command.velocity.wz) < 1e-6,
      "perpendicular startup retains a pure lateral request");
    Input corner_departure = lateral;
    corner_departure.state.pose = {.79, 0, 0};
    corner_departure.path = {{.8, 0, 0}, {.8, .8, 0}};
    corner_departure.goal = corner_departure.path.back();
    check(
      Controller(c).compute(corner_departure).command.velocity.vy > 0,
      "stationary corner departure retains a presteering intent");
    Input publication = input;
    auto scheduled = controller.compute(publication);
    check(
      controller.retime(publication, .025, scheduled), "shorter publication interval revalidated");
    check(
      std::hypot(scheduled.command.velocity.vx, scheduled.command.velocity.vy) <=
        c.limits.linear_acceleration * .025 + 1e-9,
      "publication interval bounds actual outgoing request");
    check(!controller.retime(publication, 0, scheduled), "zero publication interval rejected");
    Controller retiming_guard(c, {std::make_shared<RejectRetiming>()});
    auto guarded = retiming_guard.compute(publication);
    check(
      !retiming_guard.retime(publication, .025, guarded),
      "publication retiming cannot bypass custom hard checks");
    Controller jitter(c);
    Input timed = input;
    for (int i = 0; i < 12; ++i) {
      timed.dt = i % 2 ? .065 : .05;
      const double publication_dt = i % 2 ? .035 : .065;
      auto request = jitter.compute(timed);
      check(request.status == Status::Ok, "jitter retains a valid candidate");
      if (publication_dt < timed.dt)
        check(jitter.retime(timed, publication_dt, request), "jitter publication revalidated");
      check(
        std::hypot(
          request.command.velocity.vx - timed.state.requested.vx,
          request.command.velocity.vy - timed.state.requested.vy) <=
          c.limits.linear_acceleration * publication_dt + 1e-8,
        "variable compute latency cannot exceed publication slew");
      timed.state.requested = request.command.velocity;
    }
    Input settled = input;
    settled.path = {{0, 0, 0}};
    settled.goal = {0, 0, 0};
    settled.state.requested = {.01, 0, .05};
    const auto settled_request = Controller(c).compute(settled);
    check(
      settled_request.status == Status::Ok &&
        std::hypot(settled_request.command.velocity.vx, settled_request.command.velocity.vy) <
          1e-9 &&
        std::abs(settled_request.command.velocity.wz) < 1e-9,
      "settled terminal pose suppresses exploratory requests before Nav2 zero publication");
    check(
      Controller(c, {std::make_shared<Reject>()}).compute(settled).status == Status::NoSolution,
      "terminal hold remains subject to hard rejection");
    auto precise_settle = settled;
    precise_settle.state.requested = {};
    precise_settle.path = {{0, 0, .006}};
    precise_settle.goal = precise_settle.path.back();
    precise_settle.precise_heading = true;
    const auto precise_request = Controller(c).compute(precise_settle);
    check(
      precise_request.status == Status::Ok && precise_request.command.velocity.wz > 1e-4,
      "terminal hold must not park inside an unfinished precise heading transition");
    std::array<double, 4> lateral_boundary;
    lateral_boundary.fill(-swerve_chassis_core::pi / 2);
    const auto tiny_lateral = model.condition({-1e-9, -.0025, 0}, {}, lateral_boundary, .05);
    const auto tiny_wheels = swerve_chassis_core::inverse(
      c.geometry, {tiny_lateral.vx, tiny_lateral.vy, tiny_lateral.wz}, lateral_boundary);
    check(
      tiny_lateral.vx < 0 && tiny_wheels.steering[0] > 0,
      "conditioning preserves a tiny component that selects the lateral steering branch");
    Input alignment = input;
    alignment.path = {{0, 0, .03}};
    alignment.goal = {0, 0, .03};
    alignment.alignment = Velocity{0, 1, 0};
    auto aligned_request = Controller(c).compute(alignment);
    check(
      aligned_request.status == Status::Ok && aligned_request.command.velocity.vy > 0,
      "stopped alignment retains a feasible steering intent");
    check(
      Controller(c, {std::make_shared<Reject>()}).compute(alignment).status == Status::NoSolution,
      "explicit alignment cannot bypass injected hard rejection");
    Input cross_recovery;
    cross_recovery.state.generator.valid = true;
    cross_recovery.state.pose = {-.019, .2, .009};
    cross_recovery.state.steering.fill(swerve_chassis_core::pi / 2);
    cross_recovery.state.generator.command.steering = cross_recovery.state.steering;
    cross_recovery.path = {{0, .2, 0}, {0, 1.2, 0}};
    cross_recovery.goal = cross_recovery.path.back();
    const auto correction = Controller(c).compute(cross_recovery);
    check(
      correction.status == Status::Ok && correction.command.velocity.vx > .001,
      "small pose error with a significant steering correction must retain recovery intent");
    Input lagged_branch;
    lagged_branch.state.generator.valid = true;
    lagged_branch.state.pose = {0, 0, 1.60};
    const auto old_branch =
      swerve_chassis_core::inverse(c.geometry, {std::cos(1.60), -std::sin(1.60), .2618});
    lagged_branch.state.steering = old_branch.steering;
    lagged_branch.state.generator.command.steering = old_branch.steering;
    lagged_branch.path = {{.03, 0, 1.65}, {3, 0, 2.42754}};
    lagged_branch.goal = lagged_branch.path.back();
    lagged_branch.goal_active = false;
    Controller branch_controller(c);
    const auto transition = branch_controller.compute(lagged_branch);
    check(
      transition.status == Status::Ok && transition.branch_phase != 0,
      "cropping past a representation boundary cannot hide a measured branch mismatch");
    Controller handoff(c);
    auto handoff_input = lagged_branch;
    auto handoff_result = handoff.compute(handoff_input);
    if (handoff_result.branch_phase == 1) {
      handoff_input.state.pose.yaw = handoff_result.motion_reference->goal.yaw;
      handoff_result = handoff.compute(handoff_input);
    }
    check(handoff_result.branch_phase == 2, "handoff reaches wheel alignment");
    auto intent = *handoff_result.motion_reference->alignment;
    auto wheels = swerve_chassis_core::inverse(c.geometry, {intent.vx, intent.vy, intent.wz});
    handoff_input.state.steering = wheels.steering;
    handoff_input.state.generator.command.steering = wheels.steering;
    handoff_result = handoff.compute(handoff_input);
    check(handoff_result.branch_phase == 3, "aligned handoff departs");
    handoff_input.state.pose = handoff_result.motion_reference->goal;
    handoff_input.state.pose.yaw =
      PathView(handoff_input.path).start_match(handoff_input.state.pose).pose.yaw + .04;
    handoff_result = handoff.compute(handoff_input);
    check(handoff_result.branch_phase == 4, "measured heading error requests settling");
    handoff_input.state.pose.yaw = handoff_result.motion_reference->goal.yaw;
    wheels = swerve_chassis_core::inverse(c.geometry, {0, 0, 1});
    handoff_input.state.steering = wheels.steering;
    handoff_input.state.generator.command.steering = wheels.steering;
    handoff_result = handoff.compute(handoff_input);
    check(handoff_result.completed_transition, "measured heading settling completes");
    handoff_result = handoff.compute(handoff_input);
    check(
      handoff_result.status == Status::Ok && handoff_result.branch_phase == 0 &&
        std::hypot(handoff_result.command.velocity.vx, handoff_result.command.velocity.vy) +
            std::abs(handoff_result.command.velocity.wz) >
          1e-4,
      "spin wheel configuration must resume ordinary tracking before a new branch event");
    auto canceled_branch = lagged_branch;
    canceled_branch.path.clear();
    check(
      branch_controller.compute(canceled_branch).status == Status::InvalidInput,
      "retained transition cannot replace an invalid or canceled caller route");
    Input idle = input;
    idle.goal_active = false;
    idle.state.steering.fill(.07);
    idle.state.generator.command.steering.fill(.07);
    check(
      Controller(c).compute(idle).command.velocity.vx > 0,
      "zero request cannot impersonate a paused presteering reference");
    Trajectory on_path, shortcut;
    on_path.states = {lateral.state, lateral.state};
    on_path.states.back().pose = {0, .05, 0};
    on_path.controls = {{0, .5, 0}};
    shortcut = on_path;
    shortcut.states.back().pose = {.08, .07, 0};
    std::vector<double> costs(2, 0);
    tracking_critics().front()->score({lateral, c}, {on_path, shortcut}, costs);
    check(
      std::isfinite(costs[0]) && std::isfinite(costs[1]) && costs[1] > costs[0] + 5,
      "tracking slack is finite but shortcuts remain expensive");
    Input recovery = input;
    recovery.state.pose.yaw = -.15;
    auto recovered = Controller(c).compute(recovery);
    check(recovered.status == Status::Ok, "heading deviation retains recovery candidates");
    Input moving = input;
    moving.state.requested = moving.state.velocity = {.3, 0, 0};
    moving.state.wheel_speeds.fill(3);
    moving.state.generator.command.speed.fill(3);
    auto brake = Controller(c, {std::make_shared<BrakeOnly>()}).compute(moving);
    check(
      brake.status == Status::Ok && brake.braking_selected && brake.command.velocity.vx < .3 &&
        brake.command.velocity.vx > .25,
      "fresh validated brake preserves request slew");
    Controller rejecting(c, {std::make_shared<Reject>()});
    auto rejected = rejecting.compute(input);
    check(
      rejected.status == Status::NoSolution && rejected.diagnostics.critic_rejected == c.samples,
      "no solution reports critic rejection counts");
    auto exception = Controller(c, {std::make_shared<ThrowingCritic>()}).compute(input);
    check(
      exception.status == Status::NoSolution && exception.diagnostics.critic_errors == 1 &&
        exception.diagnostics.controller_errors == 0,
      "critic exceptions are distinguished from controller errors");
    auto extreme = input;
    extreme.path = {
      {std::numeric_limits<double>::max(), 0, 0}, {-std::numeric_limits<double>::max(), 0, 0}};
    check(
      controller.compute(extreme).status == Status::InvalidInput,
      "overflowing reference arc length is rejected");
    auto invalid_snapshot = state;
    invalid_snapshot.generator.valid = false;
    check(
      model.rollout(invalid_snapshot, {{.2, 0, 0}}).rejection == Rejection::InvalidState,
      "model rejection identifies invalid snapshots");
    c.max_speed = 0;
    Controller stopped(c);
    check(stopped.compute(input).command.velocity.vx == 0, "zero speed limit");
    c.temperature = 0;
    bool threw = false;
    try {
      Controller invalid(c);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    check(threw, "config validation");
    std::cout << "core tests passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
