#pragma once
#include "planning/detail/mode_manager.hpp"
#include "swerve_mppi/navigation/navigation.hpp"
#include "swerve_mppi/planning/optimizer.hpp"

namespace swerve_mppi::detail
{
class Planner
{
public:
  explicit Planner(
    const Config & config, std::shared_ptr<const TrajectoryValidator> validator = nullptr,
    PlanningBudget::Now now = {});
  Prediction compute(const ControllerInput & input);
  void reset();
  TransitionPhase transition_phase() const { return mode_manager_.phase(); }

private:
  Prediction compute_impl(const ControllerInput & input, const PlanningBudget & budget);
  Prediction compute_tracking(const ControllerInput & input, const PlanningBudget & budget);
  Prediction compute_goal(const ControllerInput & input, const GoalState & goal);
  Prediction request_mode(const ControllerInput & input, DriveMode mode, const Control & intent);
  Prediction apply_control(const ControllerInput & input, Control control);
  Prediction continue_alignment(const ControllerInput & input);
  Prediction planning_stop(const ControllerInput & input);
  Prediction check_stopping(const ControllerInput & input, Prediction out);
  bool safe_control(const ControllerInput & input, const Branch & branch, const Control & control);
  std::optional<Control> safe_reduction(
    const ControllerInput & input, const Branch & branch, Control control);
  std::shared_ptr<const TrajectoryValidator> validator_;
  RolloutEngine safety_rollout_;
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
  std::array<double, 4> alignment_targets_{};
  std::optional<Control> last_drive_control_;
  std::array<double, 4> last_drive_alignment_{};
  DriveMode last_drive_mode_ = DriveMode::DualAckermann;
  DriveMode alignment_mode_ = DriveMode::DualAckermann;
  double alignment_start_s_ = 0.0;
  std::size_t safety_reductions_ = 0;
};
}  // namespace swerve_mppi::detail
