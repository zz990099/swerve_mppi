#pragma once

#include "swerve_mppi/navigation.hpp"
#include "swerve_mppi/optimizer.hpp"

namespace swerve_mppi::detail {
class Planner {
public:
  explicit Planner(const Config &config,
                   std::shared_ptr<const TrajectoryValidator> validator = nullptr,
                   PlanningBudget::Now now = {});
  JointCommand compute(const ControllerInput &input);
  void reset();
  TransitionPhase transition_phase() const { return mode_manager_.phase(); }

private:
  JointCommand compute_impl(const ControllerInput &input, const PlanningBudget &budget);
  JointCommand compute_tracking(const ControllerInput &input, const PlanningBudget &budget);
  JointCommand compute_goal(const ControllerInput &input, const GoalState &goal);
  JointCommand request_mode(const ControllerInput &input, DriveMode mode, const Control &intent);
  JointCommand apply_control(const ControllerInput &input, Control control);
  JointCommand continue_alignment(const ControllerInput &input);
  JointCommand planning_stop(const ControllerInput &input);
  JointCommand check_stopping(const ControllerInput &input, JointCommand out);
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
} // namespace swerve_mppi::detail
