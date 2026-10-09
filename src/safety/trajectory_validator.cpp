#include "swerve_mppi/safety/trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "common/detail/spatial_index.hpp"
#include "safety/detail/validation.hpp"
#include "swerve_mppi/feedback/motion_observer.hpp"

namespace swerve_mppi
{
TrajectoryValidator::TrajectoryValidator(const Config & config) : config_(config)
{
  validate(config_);
}
void TrajectoryValidator::require_compatible(const Config & config) const
{
  validate(config);
  if (
    config.wheelbase_m != config_.wheelbase_m || config.track_m != config_.track_m ||
    config.robot_radius_m != config_.robot_radius_m ||
    config.collision_margin_m != config_.collision_margin_m ||
    config.steering_limit_rad != config_.steering_limit_rad ||
    config.max_wheel_speed_mps != config_.max_wheel_speed_mps ||
    config.max_path_points != config_.max_path_points ||
    config.max_obstacles != config_.max_obstacles ||
    config.feedback_linear_tolerance_mps != config_.feedback_linear_tolerance_mps ||
    config.feedback_angular_tolerance_radps != config_.feedback_angular_tolerance_radps) {
    throw std::invalid_argument("trajectory validator safety configuration differs from consumer");
  }
}
void TrajectoryValidator::add(std::shared_ptr<const TrajectoryConstraint> constraint)
{
  if (!constraint) {
    throw std::invalid_argument("trajectory constraint must not be null");
  }
  constraints_.push_back(std::move(constraint));
}
TrajectoryStatus TrajectoryValidator::check(
  const ControllerInput & input, const Trajectory & trajectory) const
{
  return check_indexed(input, trajectory, nullptr);
}
TrajectoryStatus TrajectoryValidator::check_indexed(
  const ControllerInput & input, const Trajectory & trajectory,
  const detail::SpatialIndex * obstacles) const
{
  try {
    if (
      !trajectory.valid || trajectory.poses.empty() ||
      trajectory.poses.size() > 4097 ||  // Absolute public trace cap, including the initial pose.
      (!obstacles && !detail::valid_input(input, config_)) ||
      (!trajectory.sweep_margins_m.empty() &&
       trajectory.sweep_margins_m.size() + 1 != trajectory.poses.size())) {
      return TrajectoryStatus::Invalid;
    }
    for (const auto & pose : trajectory.poses) {
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
        return TrajectoryStatus::Invalid;
      }
    }
    for (double margin : trajectory.sweep_margins_m) {
      if (!std::isfinite(margin) || margin < 0) {
        return TrajectoryStatus::Invalid;
      }
    }
    const auto & current = input.vehicle.pose;
    const auto & initial = trajectory.poses.front();
    constexpr double anchor_tolerance = 1e-9;
    const auto motion = MotionObserver(config_).assess(
      input.vehicle, input.planning_stamp_ns, input.motion_observation);
    const bool bounded_motion = motion.status == MotionStatus::NominalAgreement ||
                                motion.status == MotionStatus::BoundedDisagreement;
    const double residual_growth_mps =
      bounded_motion ? motion.linear_residual_upper_mps +
                         config_.robot_radius_m * motion.angular_residual_upper_radps
                     : 0.0;
    const double yaw_error = angle_distance(initial.yaw, current.yaw);
    if (
      !std::isfinite(yaw_error) ||
      std::hypot(initial.x - current.x, initial.y - current.y) > anchor_tolerance ||
      std::abs(yaw_error) > anchor_tolerance) {
      return TrajectoryStatus::Invalid;
    }
    const auto check_segment = [&](const Pose2d & from, const Pose2d & to, double margin) {
      // Even an obstacle-free public trace must have representable derivatives.
      detail::finite_geometry(to.x - from.x);
      detail::finite_geometry(to.y - from.y);
      const double padding =
        detail::finite_geometry(config_.robot_radius_m + config_.collision_margin_m + margin);
      const auto query = detail::Bounds::segment(from, to, padding);
      auto status = TrajectoryStatus::Valid;
      const auto inspect = [&](std::size_t index) {
        const auto & o = input.obstacles[index];
        if (
          !obstacles &&
          !query.overlaps(detail::Bounds::segment({o.x, o.y, 0}, {o.x, o.y, 0}, o.radius))) {
          return true;
        }
        const auto match = detail::segment_distance({o.x, o.y, 0}, from, to);
        const double clearance = detail::finite_geometry(match.distance - o.radius - padding);
        if (clearance <= 0) {
          status = TrajectoryStatus::Collision;
          return false;
        }
        return true;
      };
      if (obstacles) {
        obstacles->visit(query, inspect);
      } else {
        for (std::size_t j = 0; j < input.obstacles.size() && inspect(j); ++j) {
        }
      }
      return status;
    };
    // Check the exact measured footprint even when the anchor differs by roundoff.
    auto status = check_segment(current, current, 0);
    if (status != TrajectoryStatus::Valid) {
      return status;
    }
    for (std::size_t i = 0; i < trajectory.poses.size(); ++i) {
      const double margin =
        i == 0 || trajectory.sweep_margins_m.empty() ? 0 : trajectory.sweep_margins_m[i - 1];
      const double uncertainty_margin = i * config_.model_period_s * residual_growth_mps;
      status = check_segment(
        trajectory.poses[i == 0 ? 0 : i - 1], trajectory.poses[i], margin + uncertainty_margin);
      if (status != TrajectoryStatus::Valid) {
        return status;
      }
    }
    for (const auto & constraint : constraints_) {
      if (!constraint->allows(input, trajectory)) {
        return TrajectoryStatus::Rejected;
      }
    }
    return TrajectoryStatus::Valid;
  } catch (const std::invalid_argument &) {
    return TrajectoryStatus::Invalid;
  }
}
}  // namespace swerve_mppi
