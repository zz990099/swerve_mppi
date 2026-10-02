#pragma once

#include "swerve_mppi/navigation.hpp"
#include "swerve_mppi/optimizer.hpp"

namespace swerve_mppi {
class Controller {
public:
  explicit Controller(const Config &config,
                      std::shared_ptr<const TrajectoryValidator> validator = nullptr,
                      PlanningBudget::Now now = {});
  Output compute(const ControllerInput &input);
  void reset();
  TransitionPhase transition_phase() const { return mode_manager_.phase(); }

private:
  Output compute_impl(const ControllerInput &input, const PlanningBudget &budget);
  Output compute_tracking(const ControllerInput &input, const PlanningBudget &budget);
  Output compute_goal(const ControllerInput &input, const GoalState &goal);
  Output request_mode(const ControllerInput &input, DriveMode mode, const Control &intent);
  Output apply_control(const ControllerInput &input, Control control);
  Output continue_alignment(const ControllerInput &input);
  Output planning_stop(const ControllerInput &input);
  Output check_stopping(const ControllerInput &input, Output out);
  bool safe_control(const ControllerInput &input, const Branch &branch, const Control &control);
  std::optional<Control> safe_reduction(const ControllerInput &input, const Branch &branch,
                                        Control control);
  std::shared_ptr<const TrajectoryValidator> validator_;
  RolloutEngine safety_rollout_;
  ActuationModel safety_actuation_;
  Trajectory safety_trace_;
  Config config_;
  PlanningBudget::Now now_;
  DriveModel model_;
  Optimizer optimizer_;
  ModeScheduler scheduler_;
  ModeManager mode_manager_;
  PathManager path_manager_;
  GoalManager goal_manager_;
  double last_stamp_s_ = -1.0;
  std::optional<Control> alignment_control_;
  DriveMode alignment_mode_ = DriveMode::DualAckermann;
  double alignment_start_s_ = 0.0;
  std::size_t safety_reductions_ = 0;
};
} // namespace swerve_mppi
