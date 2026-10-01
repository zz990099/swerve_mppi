#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/executor.hpp"
#include "swerve_mppi/timing.hpp"
int main() {
  swerve_mppi::Controller controller(swerve_mppi::Config{});
  swerve_mppi::ModeExecutor executor(swerve_mppi::Config{});
  swerve_mppi::Output hold;
  hold.action = swerve_mppi::Action::Hold;
  auto result = executor.update(hold, {});
  swerve_mppi::TimedExecutor timed(swerve_mppi::Config{}, 1);
  const auto guarded = timed.update(swerve_mppi::CommandEnvelope{1, 1, 0, hold}, {}, 0);
  swerve_mppi::TrajectoryValidator validator(swerve_mppi::Config{});
  swerve_mppi::Trajectory stop;
  swerve_mppi::RolloutEngine(swerve_mppi::Config{}).generate_stop({}, stop);
  swerve_mppi::PathManager paths(swerve_mppi::Config{});
  swerve_mppi::ControllerInput input;
  input.reference_path = {{0, 0, 0}};
  const auto path = paths.update(input);
  return controller.compute({}).action == swerve_mppi::Action::SafeStop &&
                 result.feedback.confirmed &&
                 guarded.timing_error == swerve_mppi::TimingError::None && stop.valid &&
                 path.goal_eligible && path.target_kind == swerve_mppi::PathTargetKind::Goal &&
                 validator.check({}, {}) == swerve_mppi::TrajectoryStatus::Invalid
             ? 0
             : 1;
}
