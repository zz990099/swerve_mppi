#include <iostream>
#include <type_traits>
#include <utility>

#include "behavior_fixture.hpp"
#include "swerve_mppi/planning/controller.hpp"

using namespace swerve_mppi;
using namespace swerve_mppi::test;
namespace
{
template <class T, class = void>
struct has_action : std::false_type
{
};
template <class T>
struct has_action<T, std::void_t<decltype(std::declval<T>().action)>> : std::true_type
{
};
template <class T, class = void>
struct has_joints : std::false_type
{
};
template <class T>
struct has_joints<T, std::void_t<decltype(std::declval<T>().steering_targets)>> : std::true_type
{
};
static_assert(!has_action<Output>::value && !has_action<ChassisCommand>::value);
static_assert(
  !has_joints<Output>::value && !has_joints<ChassisCommand>::value &&
  !has_joints<ModeRequest>::value);
void test_public_velocity_is_nominal_intent()
{
  Config c;
  c.compute_budget_ratio = 0;
  const auto input = scenario_input("straight");
  const auto output = Controller(c).compute(input);
  check(
    output.command && !output.command->mode_request && output.command->target_velocity.vx > 0,
    "straight task publishes a positive body target");
  const auto & v = output.command->target_velocity;
  const auto next = DriveModel(c).step(input.vehicle, {v.vx, v.vy, v.wz}, c.model_period_s);
  check(
    next.valid && v.vx > next.state.velocity.vx,
    "public velocity is nominal intent, not the rate-limited joint endpoint");
}

void test_mode_requests_wait_for_measured_confirmation()
{
  Config c;
  c.compute_budget_ratio = 0;
  auto input = scenario_input("lateral");
  Controller controller(c);
  const auto first = controller.compute(input);
  check(first.command && first.command->mode_request, "lateral task requests a mode");
  const auto request = *first.command->mode_request;
  for (int i = 1; i < 5; ++i) {
    input.vehicle.stamp_ns = *add_duration(input.vehicle.stamp_ns, c.model_period_s);
    input.planning_stamp_ns = input.vehicle.stamp_ns;
    const auto retry = controller.compute(input);
    check(
      retry.command && retry.command->mode_request &&
        retry.command->mode_request->id == request.id &&
        retry.command->mode_request->entry_velocity.vy == request.entry_velocity.vy &&
        retry.command->target_velocity.vx == 0 && retry.command->target_velocity.vy == 0 &&
        retry.command->target_velocity.wz == 0,
      "unconfirmed requests remain frozen zero-motion intents");
  }
  input.vehicle.mode_fault = true;
  input.vehicle.stamp_ns = *add_duration(input.vehicle.stamp_ns, c.model_period_s);
  input.planning_stamp_ns = input.vehicle.stamp_ns;
  check(!controller.compute(input).command, "fault must withhold authorization");
}
void test_zero_and_invalid_output_are_distinct()
{
  Config c;
  c.compute_budget_ratio = 0;
  ControllerInput input;
  input.reference_path = {{0, 0, 0}};
  const auto hold = Controller(c).compute(input);
  check(
    hold.command && !hold.command->mode_request &&
      hold.command->mode == input.vehicle.actual_mode && hold.command->target_velocity.vx == 0 &&
      hold.command->target_velocity.vy == 0 && hold.command->target_velocity.wz == 0,
    "valid zero holds the current mode");
  check(!Controller(c).compute({}).command, "invalid task has no command");
}
}  // namespace
int main()
{
  try {
    test_public_velocity_is_nominal_intent();
    test_mode_requests_wait_for_measured_confirmation();
    test_zero_and_invalid_output_are_distinct();
    std::cout << "Public chassis command boundary passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
