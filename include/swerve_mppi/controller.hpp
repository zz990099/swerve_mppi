#pragma once

#include "swerve_mppi/optimizer.hpp"

namespace swerve_mppi {
class Controller {
public:
  explicit Controller(const Config &config);
  Output compute(const ControllerInput &input);
  void reset();
  TransitionPhase transition_phase() const { return mode_manager_.phase(); }

private:
  Output continue_alignment(const ControllerInput &input);
  Config config_;
  DriveModel model_;
  Optimizer optimizer_;
  ModeScheduler scheduler_;
  ModeManager mode_manager_;
  double last_stamp_s_ = -1.0;
  std::optional<Control> alignment_control_;
  DriveMode alignment_mode_ = DriveMode::DualAckermann;
  double alignment_start_s_ = 0.0;
};
} // namespace swerve_mppi
