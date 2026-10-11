#pragma once

#include "swerve_mppi/common/config.hpp"
#include "swerve_mppi/common/types.hpp"

namespace swerve_mppi
{

double wrap_angle(double angle);
double angle_distance(double a, double b);
// Handover threshold only; residual wheel speeds still require complete
// braking.
bool is_stopped(const VehicleState & state, const Config & config);

struct WheelCommand
{
  std::array<double, 4> angles{};
  std::array<double, 4> speeds{};
  bool valid = true;
};

// Bounded inverse and encoder forward kinematics; order FL, FR, RL, RR.
class Kinematics
{
public:
  explicit Kinematics(const Config & config);
  WheelCommand inverse(const Control & control, const std::array<double, 4> & current_angles) const;
  Twist2d forward(const std::array<double, 4> & speeds, const std::array<double, 4> & angles) const;

  // Largest module rolling-vector error against one rigid-body velocity field.
  double max_module_residual(
    const std::array<double, 4> & speeds, const std::array<double, 4> & angles,
    const Twist2d & twist) const;

private:
  Config config_;
};

// Command-side prediction only. Never substitute it for measured feedback.
// Cold seeds assume commanded joints and limited velocity equal the observation;
// carry this context between steps for a known nominal command history.
struct ChassisPrediction
{
  TransitionPhase phase = TransitionPhase::Stable;
  Twist2d limited_velocity;
  std::array<double, 4> commanded_angles{};
  std::array<double, 4> commanded_wheel_radps{};
  std::array<double, 4> alignment{};
  TimestampNs transition_start_ns = 0;
  TimestampNs aligned_since_ns = kInvalidTimestamp;
};

struct StepResult
{
  VehicleState state;
  std::array<double, 4> steering_targets{};
  std::array<double, 4> wheel_speed_targets{};
  bool valid = true;
  bool aligning = false;
  ChassisPrediction prediction;
  // Conservative enclosure for the complete sample-and-hold nominal motion.
  double sweep_margin_m = 0;
  double integration_error_m = 0;
};

class DriveModel
{
public:
  explicit DriveModel(const Config & config);

  bool feasible(const Control & control, DriveMode mode) const;
  Control project(const Control & control, DriveMode mode) const;
  // Cold prediction from an observation; this is an explicit nominal assumption.
  StepResult step(const VehicleState & start, const Control & control, double model_period_s) const;
  StepResult step(
    const VehicleState & start, const Control & control, double model_period_s,
    const ChassisPrediction & prediction) const;
  ChassisPrediction seed(const VehicleState & observed) const;
  ChassisPrediction alignment_seed(
    const VehicleState & observed, const std::array<double, 4> & frozen_angles) const;
  // Shared chassis caps; planner projection remains a separate policy.
  Control bounded(const Control & intent) const;
  std::array<double, 4> steering_for_mode(
    DriveMode mode, const std::array<double, 4> & current_angles = {}) const;
  std::array<double, 4> steering_for_entry(
    DriveMode mode, const Control & intent, const std::array<double, 4> & current_angles) const;

private:
  Config config_;
  Kinematics kinematics_;
};

// Predicts braking, steering alignment, and the time spent waiting for a mode
// confirmation. Real execution is gated separately on the actual feedback.
class TransitionModel
{
public:
  explicit TransitionModel(const Config & config);

  double rollout(
    VehicleState & state, DriveMode target_mode, std::size_t & steps_used, std::size_t max_steps,
    std::vector<Pose2d> * trace = nullptr, const Control & entry_intent = {},
    std::vector<double> * sweep_margins = nullptr, double * position_error_m = nullptr) const;

private:
  Config config_;
  DriveModel model_;
};

}  // namespace swerve_mppi
