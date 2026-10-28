#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "swerve_mppi/controller.hpp"
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
int main()
{
  try {
    Config c;
    Model model(c);
    State state;
    state.generator.valid = true;
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
    Controller rejecting(c, {std::make_shared<Reject>()});
    check(rejecting.compute(input).status == Status::NoSolution, "no solution");
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
