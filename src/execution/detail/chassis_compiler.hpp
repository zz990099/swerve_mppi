#pragma once
#include "safety/detail/validation.hpp"
#include "swerve_mppi/feedback/feedback.hpp"
#include "swerve_mppi/model/model.hpp"
namespace swerve_mppi::detail
{
// Compiles a body target at the actual execution snapshot using the SAME model
// used in prediction. Copy/commit this cache transactionally with admission.
class ChassisCompiler
{
public:
  explicit ChassisCompiler(const Config & config) : config_(config), model_(config) {}
  JointCommand compile(const Output & output, const VehicleState & measured)
  {
    JointCommand out;
    out.requested_mode = measured.actual_mode;
    if (
      check_model_feedback(measured, config_).status != FeedbackStatus::Valid ||
      measured.mode_fault || !output.command) {
      return out;
    }
    const auto & command = *output.command;
    const auto & v = command.target_velocity;
    const Control intent{v.vx, v.vy, v.wz};
    if (!valid_mode(command.mode) || !model_.feasible(intent, command.mode)) {
      return out;
    }
    out.steering_targets = measured.steering_angles;
    if (command.mode_request) {
      const auto & r = *command.mode_request;
      const auto & entry = r.entry_velocity;
      if (
        v.vx != 0 || v.vy != 0 || v.wz != 0 || r.id == 0 || command.mode != r.mode ||
        !valid_mode(r.mode) || !model_.feasible({entry.vx, entry.vy, entry.wz}, r.mode)) {
        return out;
      }
      if (request_ && r.id == request_->id) {
        if (
          r.mode != request_->mode || entry.vx != request_->entry_velocity.vx ||
          entry.vy != request_->entry_velocity.vy || entry.wz != request_->entry_velocity.wz) {
          return out;
        }
      } else {
        if (request_ && r.id < request_->id) {
          return out;
        }
        request_ = r;
        joint_request_ = {
          r.id, r.mode,
          model_.steering_for_entry(
            r.mode, {entry.vx, entry.vy, entry.wz}, measured.steering_angles),
          entry};
      }
      out.action = Action::RequestMode;
      out.requested_mode = r.mode;
      out.mode_request = joint_request_;
      out.steering_targets = joint_request_.steering_targets;
      return out;
    }
    // A velocity packet cannot silently switch the selected chassis mode.
    if (command.mode != measured.actual_mode) {
      return out;
    }
    if (v.vx == 0 && v.vy == 0 && v.wz == 0) {
      out.action = is_stopped(measured, config_) ? Action::Hold : Action::Brake;
      return out;
    }
    if (!measured.mode_confirmed) {
      return out;
    }
    const auto next = model_.step(measured, intent, config_.dt_s);
    if (!next.valid) {
      return out;
    }
    out.velocity_intent = v;
    out.steering_targets = next.steering_targets;
    if (next.aligning) {
      out.action = is_stopped(measured, config_) ? Action::Hold : Action::Brake;
    } else {
      out.action = Action::Drive;
      out.body_command = next.state.velocity;
      out.wheel_speed_targets = next.wheel_speed_targets;
    }
    return out;
  }
  void reset() { request_.reset(); }

private:
  Config config_;
  DriveModel model_;
  std::optional<ModeRequest> request_;
  JointModeRequest joint_request_;
};
}  // namespace swerve_mppi::detail
