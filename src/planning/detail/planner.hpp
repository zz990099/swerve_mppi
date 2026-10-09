#pragma once
#include "planning/detail/mode_manager.hpp"
#include "swerve_mppi/feedback/motion_observer.hpp"
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
  bool recover(const ControllerInput & input);
  std::uint64_t record_command(
    const ControllerInput & input, const ChassisCommand & command, TimestampNs computed_stamp_ns,
    TimestampNs valid_until_ns);
  TimestampNs command_expiry(TimestampNs computed_stamp_ns) const
  {
    return add_duration(computed_stamp_ns, config_.command_lifetime_s).value_or(kInvalidTimestamp);
  }
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
  bool temporal_admission(const ControllerInput & input, Prediction & stop);
  bool motion_admission(const ControllerInput & input, Prediction & stop);
  PredictionHistoryStatus synchronize_history(const ControllerInput & input);
  void latch(FailureReason reason);
  std::optional<Control> safe_reduction(
    const ControllerInput & input, const Branch & branch, Control control);
  struct IssuedCommand
  {
    std::uint64_t id = 0;
    VehicleState observation;
    ChassisCommand command;
    TimestampNs computed_stamp_ns = kInvalidTimestamp;
    TimestampNs valid_until_ns = kInvalidTimestamp;
    ChassisPrediction prediction;
    Control pre_application_control;
    bool prediction_known = false;
  };
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
  TimestampNs last_stamp_ns_ = kInvalidTimestamp;
  std::optional<Control> alignment_control_;
  std::array<double, 4> alignment_targets_{};
  std::optional<Control> last_drive_control_;
  std::array<double, 4> last_drive_alignment_{};
  DriveMode last_drive_mode_ = DriveMode::DualAckermann;
  DriveMode alignment_mode_ = DriveMode::DualAckermann;
  TimestampNs alignment_start_ns_ = 0;
  TimestampNs last_planning_stamp_ns_ = kInvalidTimestamp;
  TimestampNs warm_start_stamp_ns_ = kInvalidTimestamp;
  std::uint64_t last_command_id_ = 0;
  std::optional<IssuedCommand> issued_command_;
  std::optional<ChassisPrediction> synchronized_prediction_;
  std::optional<Control> active_control_;
  MotionAssessment motion_assessment_;
  PredictionHistoryStatus prediction_history_status_ = PredictionHistoryStatus::ColdStart;
  bool fault_latched_ = false;
  FailureReason latched_reason_ = FailureReason::None;
  std::size_t safety_reductions_ = 0;
};
}  // namespace swerve_mppi::detail
