#pragma once

#include "swerve_mppi/model.hpp"

namespace swerve_mppi {
struct Branch {
  DriveMode mode = DriveMode::DualAckermann;
  std::size_t switch_step = 0;
  bool switches = false;
};
struct Trajectory {
  Branch branch;
  std::vector<Pose2d> poses; // Initial pose followed by one pose per model tick.
  std::vector<Control> controls;
  std::vector<bool> active_controls; // False for proposals ignored by a committed transition.
  VehicleState final_state;
  bool valid = false;
};
class RolloutEngine {
public:
  explicit RolloutEngine(const Config &config);
  Trajectory generate(const VehicleState &initial, const Branch &branch,
                      const std::vector<Control> &controls) const;

  // Reuse caller-owned vector capacity. Controls must not alias out.controls.
  void generate(const VehicleState &initial, const Branch &branch,
                const std::vector<Control> &controls, Trajectory &out) const;

  // Zero drive with measured steering retained. Braking must remain checkable
  // during an unconfirmed mode transition; it never synthesizes confirmation.
  void generate_stop(const VehicleState &initial, Trajectory &out) const;

  // Commit entry/alignment through the first Drive, then brake to zero. Uses
  // stopping_horizon_steps rather than the optimization horizon; fails closed.
  void generate_continuation(const VehicleState &initial, const Branch &branch,
                             const Control &first_control, Trajectory &out) const;

private:
  void stopping_rollout(const VehicleState &initial, const Branch &branch,
                        const Control *first_control, Trajectory &out) const;
  Config config_;
  DriveModel model_;
  TransitionModel transition_;
};
} // namespace swerve_mppi
