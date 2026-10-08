#pragma once
#include <algorithm>
#include <cmath>

#include "planning/detail/prediction.hpp"
#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi::test
{
// Test-only nominal target generator. Independent encoder integration lives in
// behavior_fixture.hpp. Reusing DriveModel for target compilation deliberately
// preserves stage-1 nominal regressions; this is not the Python/Gazebo plant.
struct PlantTargets
{
  detail::Action action = detail::Action::Hold;
  std::array<double, 4> steering_targets{};
  std::array<double, 4> wheel_speed_targets{};
  ModeFeedback feedback;
};
class NominalChassis
{
public:
  explicit NominalChassis(const Config & config) : config_(config), model_(config) {}

  PlantTargets update(const Output & output, const VehicleState & measured)
  {
    using detail::Action;
    PlantTargets out;
    out.steering_targets = measured.steering_angles;
    out.feedback = {
      measured.actual_mode,
      measured.mode_confirmed,
      measured.mode_fault,
      measured.mode_request_id,
      measured.time_in_mode_s + config_.dt_s,
      measured.accepted_mode_request};
    auto reject = [&]() {
      fault_ = true;
      out.action = Action::SafeStop;
      out.feedback.fault = true;
      out.feedback.confirmed = false;
      return out;
    };
    if (fault_ || measured.mode_fault || !output.command) return reject();
    const auto & command = *output.command;
    if (command.mode_request) {
      const auto & r = *command.mode_request;
      if (!accepted_ || r.id != accepted_->id) {
        if (pending_ || r.id <= measured.mode_request_id) return reject();
        accepted_ = AcceptedModeRequest{
          r.id, r.mode,
          model_.steering_for_entry(
            r.mode, {r.entry_velocity.vx, r.entry_velocity.vy, r.entry_velocity.wz},
            measured.steering_angles),
          r.entry_velocity};
        pending_ = true;
        aligned_since_ = -1;
      }
      if (
        r.mode != accepted_->mode || r.entry_velocity.vx != accepted_->entry_velocity.vx ||
        r.entry_velocity.vy != accepted_->entry_velocity.vy ||
        r.entry_velocity.wz != accepted_->entry_velocity.wz)
        return reject();
      out.feedback.request_id = accepted_->id;
      out.feedback.accepted_mode_request = accepted_;
    }
    if (pending_) {
      out.feedback.confirmed = false;
      out.action = Action::Brake;
      if (is_stopped(measured, config_)) {
        out.action = Action::RequestMode;
        out.steering_targets = accepted_->steering_targets;
        bool aligned = true;
        for (std::size_t i = 0; i < 4; ++i) {
          aligned = aligned && std::abs(measured.steering_angles[i] - out.steering_targets[i]) <=
                                 config_.steering_tolerance_rad;
        }
        if (aligned) {
          if (aligned_since_ < 0) aligned_since_ = measured.stamp_s;
          if (measured.stamp_s - aligned_since_ + 1e-12 >= config_.alignment_min_s) {
            pending_ = false;
            out.feedback.actual_mode = accepted_->mode;
            out.feedback.confirmed = true;
            out.feedback.time_in_mode_s = 0;
            out.action = Action::Hold;
          }
        } else
          aligned_since_ = -1;
      }
      return out;
    }
    const auto & v = command.target_velocity;
    if (v.vx == 0 && v.vy == 0 && v.wz == 0) {
      out.action = is_stopped(measured, config_) ? Action::Hold : Action::Brake;
      return out;
    }
    if (command.mode != measured.actual_mode || !measured.mode_confirmed) return reject();
    const auto target = model_.step(measured, {v.vx, v.vy, v.wz}, config_.dt_s);
    if (!target.valid) return reject();
    out.steering_targets = target.steering_targets;
    out.action = target.aligning ? (is_stopped(measured, config_) ? Action::Hold : Action::Brake)
                                 : Action::Drive;
    if (!target.aligning) out.wheel_speed_targets = target.wheel_speed_targets;
    return out;
  }

  // Private planner regressions traverse the same body-command boundary as Controller.
  PlantTargets update(const detail::Prediction & prediction, const VehicleState & measured)
  {
    Output output;
    if (prediction.action != detail::Action::SafeStop) {
      output.command = ChassisCommand{measured.actual_mode, prediction.velocity_intent, {}};
      if (prediction.action == detail::Action::RequestMode && prediction.mode_request) {
        const auto & r = *prediction.mode_request;
        output.command = ChassisCommand{r.mode, {}, ModeRequest{r.id, r.mode, r.entry_velocity}};
      }
    }
    return update(output, measured);
  }

private:
  Config config_;
  DriveModel model_;
  std::optional<AcceptedModeRequest> accepted_;
  bool pending_ = false;
  bool fault_ = false;
  double aligned_since_ = -1;
};
}  // namespace swerve_mppi::test
