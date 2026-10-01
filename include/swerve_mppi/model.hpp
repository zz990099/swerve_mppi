#pragma once

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/types.hpp"

namespace swerve_mppi {

double wrap_angle(double angle);
double angle_distance(double a, double b);
bool is_stopped(const VehicleState &state, const Config &config);

struct WheelCommand {
  std::array<double, 4> angles{};
  std::array<double, 4> speeds{};
  bool valid = true;
};

// Bounded inverse and encoder forward kinematics; order FL, FR, RL, RR.
class Kinematics {
public:
  explicit Kinematics(const Config &config);
  WheelCommand inverse(const Control &control, const std::array<double, 4> &current_angles) const;
  Twist2d forward(const std::array<double, 4> &speeds, const std::array<double, 4> &angles) const;

  // Largest module rolling-vector error against one rigid-body velocity field.
  double max_module_residual(const std::array<double, 4> &speeds,
                             const std::array<double, 4> &angles, const Twist2d &twist) const;

private:
  Config config_;
};

struct StepResult {
  VehicleState state;
  std::array<double, 4> steering_targets{};
  std::array<double, 4> wheel_speed_targets{};
  bool valid = true;
  bool aligning = false;
  // Conservative enclosure for the modeled within-tick body-twist ramp.
  double sweep_margin_m = 0;
  double integration_error_m = 0;
};

class DriveModel {
public:
  explicit DriveModel(const Config &config);

  bool feasible(const Control &control, DriveMode mode) const;
  Control project(const Control &control, DriveMode mode) const;
  StepResult step(const VehicleState &start, const Control &control, double dt_s) const;
  std::array<double, 4> steering_for_mode(DriveMode mode,
                                          const std::array<double, 4> &current_angles = {}) const;
  std::array<double, 4> steering_for_entry(DriveMode mode, const Control &intent,
                                           const std::array<double, 4> &current_angles) const;

private:
  Config config_;
  Kinematics kinematics_;
};

// Predicts braking, steering alignment, and the time spent waiting for a mode
// confirmation. Real execution is gated separately on the actual feedback.
class TransitionModel {
public:
  explicit TransitionModel(const Config &config);

  double rollout(VehicleState &state, DriveMode target_mode, std::size_t &steps_used,
                 std::size_t max_steps, std::vector<Pose2d> *trace = nullptr,
                 const Control &entry_intent = {}, std::vector<double> *sweep_margins = nullptr,
                 double *position_error_m = nullptr) const;

private:
  Config config_;
  DriveModel model_;
};

} // namespace swerve_mppi
