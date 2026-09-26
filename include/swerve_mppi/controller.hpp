#pragma once

#include "swerve_mppi/optimizer.hpp"

namespace swerve_mppi {

class ModeManager {
 public:
  explicit ModeManager(const Config & config) : config_(config) {}

  void begin(DriveMode target_mode, double now_s);
  bool active() const { return phase_ != TransitionPhase::Stable; }
  TransitionPhase phase() const { return phase_; }
  Output update(const VehicleState & observed);
  void reset();

 private:
  const Config & config_;
  TransitionPhase phase_ = TransitionPhase::Stable;
  DriveMode target_mode_ = DriveMode::DualAckermann;
  double start_s_ = 0.0;
};

class Controller {
 public:
  explicit Controller(const Config & config);
  Output compute(const ControllerInput & input);
  void reset();
  TransitionPhase transition_phase() const { return mode_manager_.phase(); }

 private:
  Config config_;
  DriveModel model_;
  Optimizer optimizer_;
  ModeManager mode_manager_;
  double last_stamp_s_ = -1.0;
};

}  // namespace swerve_mppi
