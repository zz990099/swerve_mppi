#pragma once
#include <algorithm>
#include <cmath>

#include "planning/detail/prediction.hpp"
#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi::test
{
// Test-only nominal target generator. Independent encoder integration lives in
// behavior_fixture.hpp. Reusing DriveModel for target compilation deliberately
// matches the command model; direct Python parity is a separate test.
// This fixture is not the physical Gazebo servo/contact plant.
struct JointSample
{
  std::array<double, 4> angles{};
  std::array<double, 4> speeds{};
  double dt_s = 0;
};
struct PlantTargets
{
  detail::Action action = detail::Action::Hold;
  std::array<double, 4> steering_targets{};
  std::array<double, 4> wheel_speed_targets{};
  std::vector<JointSample> samples;
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
        begin_ = true;
      }
      if (
        r.mode != accepted_->mode || r.entry_velocity.vx != accepted_->entry_velocity.vx ||
        r.entry_velocity.vy != accepted_->entry_velocity.vy ||
        r.entry_velocity.wz != accepted_->entry_velocity.wz)
        return reject();
      out.feedback.request_id = accepted_->id;
      out.feedback.accepted_mode_request = accepted_;
    }
    if (!memory_) memory_ = model_.seed(measured);
    if (begin_) {
      memory_->phase = TransitionPhase::Braking;
      memory_->alignment = accepted_->steering_targets;
      memory_->transition_start_s = measured.stamp_s;
      memory_->aligned_since_s = -1;
      begin_ = false;
    }
    const auto & v = command.target_velocity;
    if (pending_ && (v.vx != 0 || v.vy != 0 || v.wz != 0)) return reject();
    if (!pending_ && command.mode != measured.actual_mode) return reject();
    auto state = measured;
    const auto ticks =
      static_cast<std::size_t>(std::ceil(config_.dt_s / config_.chassis_period_s - 1e-12));
    const double h = config_.dt_s / ticks;
    bool aligning = false;
    for (std::size_t tick = 0; tick < ticks; ++tick) {
      const auto next = model_.step(state, {v.vx, v.vy, v.wz}, h, *memory_);
      if (!next.valid) return reject();
      memory_ = next.prediction;
      aligning = aligning || next.aligning;
      out.samples.push_back({next.steering_targets, next.wheel_speed_targets, h});
      state = next.state;
    }
    out.steering_targets = state.steering_angles;
    out.wheel_speed_targets = state.wheel_speeds;
    if (pending_ && memory_->phase == TransitionPhase::Stable) {
      pending_ = false;
      out.feedback.actual_mode = accepted_->mode;
      out.feedback.time_in_mode_s = 0;
    }
    out.feedback.confirmed = memory_->phase == TransitionPhase::Stable && !pending_;
    out.action = aligning ? (is_stopped(measured, config_) ? Action::Hold : Action::Brake)
                 : (v.vx == 0 && v.vy == 0 && v.wz == 0)
                   ? (is_stopped(measured, config_) ? Action::Hold : Action::Brake)
                   : Action::Drive;
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
  bool begin_ = false;
  std::optional<ChassisPrediction> memory_;
};
}  // namespace swerve_mppi::test
