#pragma once

#include "swerve_mppi/config.hpp"
#include "swerve_mppi/types.hpp"

namespace swerve_mppi {

double wrap_angle(double angle);
double angle_distance(double a, double b);

struct StepResult {
  VehicleState state;
  std::array<double, 4> steering_targets{};
  std::array<double, 4> wheel_speed_targets{};
  bool valid = true;
};

class DriveModel {
 public:
  explicit DriveModel(const Config & config) : config_(config) {}

  bool feasible(const Control & control, DriveMode mode) const;
  Control project(const Control & control, DriveMode mode) const;
  StepResult step(const VehicleState & start, const Control & control,
                  double dt_s) const;
  std::array<double, 4> steering_for_mode(DriveMode mode) const;

 private:
  const Config & config_;
};

// Predicts braking, steering alignment, and the time spent waiting for a mode
// confirmation. Real execution is gated separately on the actual feedback.
class TransitionModel {
 public:
  TransitionModel(const Config & config, const DriveModel & model)
      : config_(config), model_(model) {}

  double rollout(VehicleState & state, DriveMode target_mode,
                 std::size_t & steps_used, std::size_t max_steps,
                 std::vector<Pose2d> * trace = nullptr) const;

 private:
  const Config & config_;
  const DriveModel & model_;
};

}  // namespace swerve_mppi
