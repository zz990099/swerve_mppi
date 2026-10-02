#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/executor.hpp"
#include "swerve_mppi/timing.hpp"
#include "swerve_mppi/feedback.hpp"
#include "swerve_mppi/profile_runner.hpp"
int main() {
  swerve_mppi::Controller controller(swerve_mppi::Config{});
  swerve_mppi::ModeExecutor executor(swerve_mppi::Config{});
  swerve_mppi::Output hold;
  hold.action = swerve_mppi::Action::Hold;
  auto result = executor.update(hold, {});
  swerve_mppi::TimedExecutor timed(swerve_mppi::Config{}, 1);
  swerve_mppi::ControllerInput current;
  current.reference_path = {{0, 0, 0}};
  const auto guarded = timed.update(swerve_mppi::CommandEnvelope{
      1, 1, 0, hold, 0, 0, .025, swerve_mppi::CommandTask::capture(current)}, current, 0);
  const auto midpoint = guarded.actuation ? guarded.actuation->sample(.05) : std::nullopt;
  swerve_mppi::ProfileRunner runner(swerve_mppi::Config{});
  const bool installed = runner.install(guarded, 0, 1);
  const auto joints = runner.sample(.05, 1.05);
  const auto nominal_stop = swerve_mppi::ActuationModel(swerve_mppi::Config{})
      .plan_stopping(current.vehicle, swerve_mppi::Action::Hold, {});
  swerve_mppi::TrajectoryValidator validator(swerve_mppi::Config{});
  validator.require_compatible(swerve_mppi::Config{});
  const auto residual =
      swerve_mppi::Kinematics(swerve_mppi::Config{}).max_module_residual({}, {}, {});
  swerve_mppi::NoiseGenerator noise(swerve_mppi::Config{});
  const double correction = noise.correction({}, {}, {}, {});
  const double legacy_correction = noise.correction({}, {}, {});
  swerve_mppi::Trajectory stop;
  swerve_mppi::RolloutEngine(swerve_mppi::Config{}).generate_stop({}, stop);
  swerve_mppi::Trajectory continuation;
  swerve_mppi::RolloutEngine(swerve_mppi::Config{})
      .generate_continuation({}, {}, {.1, 0, 0}, continuation);
  const auto step = swerve_mppi::DriveModel(swerve_mppi::Config{}).step({}, {.1, 0, 0}, .1);
  swerve_mppi::PathManager paths(swerve_mppi::Config{});
  swerve_mppi::ControllerInput input;
  input.reference_path = {{0, 0, 0}};
  const auto path = paths.update(input);
  return controller.compute({}).action == swerve_mppi::Action::SafeStop &&
                 result.feedback.confirmed &&
                 guarded.timing_error == swerve_mppi::TimingError::None &&
                 guarded.safety_error == swerve_mppi::ExecutionSafetyError::None && midpoint &&
                 midpoint->wheel_speeds[0] == 0 && installed && joints &&
                 swerve_mppi::check_feedback({}, swerve_mppi::Config{}).status ==
                     swerve_mppi::FeedbackStatus::Valid && nominal_stop && residual == 0 &&
                 correction == 0 && legacy_correction == 0 && stop.valid && continuation.valid &&
                 continuation.sweep_margins_m.size() + 1 == continuation.poses.size() &&
                 continuation.position_error_m >= 0 && step.sweep_margin_m >= 0 &&
                 step.integration_error_m >= 0 && path.goal_eligible &&
                 path.target_kind == swerve_mppi::PathTargetKind::Goal &&
                 validator.check({}, {}) == swerve_mppi::TrajectoryStatus::Invalid
             ? 0
             : 1;
}
