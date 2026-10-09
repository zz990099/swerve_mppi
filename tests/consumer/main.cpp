#include "swerve_mppi/common/config_profile.hpp"
#include "swerve_mppi/feedback/feedback_adapter.hpp"
#include "swerve_mppi/feedback/motion_observer.hpp"
#include "swerve_mppi/navigation/navigation.hpp"
#include "swerve_mppi/planning/controller.hpp"
#include "swerve_mppi/planning/optimizer.hpp"

#if __has_include(                                           \
  "swerve_mppi/execution/executor.hpp") ||                   \
  __has_include(                                             \
    "swerve_mppi/execution/timing.hpp") ||                   \
    __has_include(                                           \
      "swerve_mppi/execution/profile_runner.hpp") ||         \
      __has_include(                                         \
        "swerve_mppi/execution/chassis_executor.hpp") ||     \
        __has_include(                                       \
          "swerve_mppi/integration/adapter_contract.hpp") || \
          __has_include(                                     \
            "swerve_mppi/model/actuation.hpp") || __has_include("planning/detail/prediction.hpp")
#error "Installation exposes removed execution APIs or private prediction code"
#endif

int main()
{
  using namespace swerve_mppi;
  auto config = parse_config_profile("max_linear_accel_mps2=.5\nrandom_seed=7");
  const auto restored = parse_resolved_config_profile(write_config_profile(config));
  validate_live_config(restored);
  config.compute_budget_ratio = 0;
  Controller controller(config);
  ControllerInput input;
  input.reference_path = {{0, 0, 0}};
  const auto hold = controller.compute(input);
  const auto rejected = Controller(config).compute({});
  Trajectory stop;
  RolloutEngine(config).generate_stopping_interval({}, {}, stop);
  const auto motion =
    MotionObserver(config).assess({}, 0, MotionObservation{{}, 0, MotionSource::IndependentBody});
  const auto missing = FeedbackAdapter(config).make({}, {}, {}, 0);
  return hold.command && hold.command->target_velocity.vx == 0 && !hold.command->mode_request &&
             !rejected.command && stop.valid && restored.random_seed == 7 &&
             restored.max_linear_accel_mps2 == .5 &&
             motion.status == MotionStatus::NominalAgreement &&
             missing.error == SnapshotError::InvalidTime
           ? 0
           : 1;
}
