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
  void score(const CriticContext &, const std::vector<Trajectory> & t, std::vector<double> & costs)
    const override
  {
    for (std::size_t k = 0; k < t.size(); ++k) costs[k] = std::numeric_limits<double>::infinity();
  }
};
int main()
{
  try {
    Config c;
    Model model(c);
    for (auto mode : {Mode::Ackermann, Mode::Spin, Mode::Crab}) {
      auto v = model.project({4, 3, 2}, mode);
      auto w = model.inverse(v, {});
      auto f = model.forward(w);
      check(
        std::hypot(f.vx - v.vx, f.vy - v.vy) < 1e-10 && std::abs(f.wz - v.wz) < 1e-10,
        "kinematics roundtrip");
      check(std::hypot(v.vx, v.vy) <= c.max_speed + 1e-9, "speed cap");
      if (mode == Mode::Spin) check(v.vx == 0 && v.vy == 0, "spin geometry");
    }
    Config slow = c;
    slow.max_wheel_speed = 0.1;
    Model slow_model(slow);
    for (auto mode : {Mode::Ackermann, Mode::Spin, Mode::Crab}) {
      for (double sign : {-1.0, 1.0}) {
        auto command = slow_model.project({sign * 0.8, sign * 0.5, sign * 0.8}, mode);
        for (double speed : slow_model.inverse(command, {}).speeds)
          check(std::abs(speed) <= slow.max_wheel_speed + 1e-9, "projected wheel cap in all modes");
        auto trace = slow_model.rollout({}, mode, std::vector<Velocity>(slow.horizon, command));
        check(trace.valid, "low wheel limit rollout");
        for (const auto & sample : trace.states)
          for (double speed : sample.wheel_speeds)
            check(
              std::abs(speed) <= slow.max_wheel_speed + 1e-9, "predicted wheel cap in all modes");
      }
    }
    State state;
    std::vector<Velocity> controls(c.horizon, {0, 0.4, 0});
    auto t = model.rollout(state, Mode::Crab, controls);
    check(
      t.valid && t.states.back().mode == Mode::Crab && t.states.back().pose.y > 0.4,
      "mode transition followed by crab motion");
    for (std::size_t j = 1; j < t.states.size(); ++j)
      for (std::size_t i = 0; i < 4; ++i) {
        check(
          std::abs(t.states[j].steering[i] - t.states[j - 1].steering[i]) <=
            c.steer_rate * c.model_dt + 1e-8,
          "steer slew");
        check(std::abs(t.states[j].wheel_speeds[i]) <= c.max_wheel_speed + 1e-8, "wheel cap");
      }
    Input input;
    input.path = {{0, 0, 0}, {1, 0, 0}};
    input.goal = input.path.back();
    Controller controller(c);
    auto result = controller.compute(input);
    check(result.status == Status::Ok && result.command.velocity.vx > 0, "valid command");
    input.state.pose.x = std::numeric_limits<double>::quiet_NaN();
    check(controller.compute(input).status == Status::InvalidInput, "invalid input");
    input.state = {};
    check(controller.compute(input).status == Status::Ok, "next valid call requires no recovery");
    input.state.switching = true;
    input.state.requested_mode = Mode::Crab;
    auto waiting = controller.compute(input);
    check(
      waiting.status == Status::Switching && waiting.command.mode == Mode::Crab &&
        waiting.command.velocity.vx == 0,
      "transition state");
    input.state = {};
    Controller rejecting(c, {std::make_shared<Reject>()});
    check(rejecting.compute(input).status == Status::NoSolution, "injected critic");
    std::uint8_t bytes[] = {0, 254, 255, 2};
    GridView grid{bytes, 2, 2, 0.1, -1, -1};
    std::size_t x, y;
    check(grid.world_to_map(-0.85, -0.95, x, y) && x == 1 && y == 0, "grid origin");
    check(!grid.world_to_map(-1.01, -1, x, y), "grid bounds");
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
