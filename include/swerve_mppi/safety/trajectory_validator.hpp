#pragma once

#include <memory>

#include "swerve_mppi/model/rollout.hpp"

namespace swerve_mppi
{
namespace detail
{
class SpatialIndex;
}
enum class TrajectoryStatus
{
  Valid,
  Invalid,
  Collision,
  Rejected
};

// Extra hard constraints, independent of optimization cost. Implementations
// must be const, bounded and safe to share across all controller policies.
class TrajectoryConstraint
{
public:
  virtual ~TrajectoryConstraint() = default;
  virtual bool allows(const ControllerInput & input, const Trajectory & trajectory) const = 0;
};

class TrajectoryValidator
{
public:
  explicit TrajectoryValidator(const Config & config);
  void add(std::shared_ptr<const TrajectoryConstraint> constraint);
  // Reject a different footprint, margin or measured joint validity envelope.
  // Planning-only parameters may differ because check() consumes explicit
  // poses.
  void require_compatible(const Config & config) const;
  // The initial pose must match input.vehicle.pose within 1e-9 m/rad
  // (yaw modulo 2*pi). Also checks the exact current circular footprint.
  TrajectoryStatus check(const ControllerInput & input, const Trajectory & trajectory) const;

private:
  friend class CriticManager;
  // Internal prepared contexts have already passed complete input validation.
  // Public check() always validates its own input and never reuses this token.
  TrajectoryStatus check_indexed(
    const ControllerInput & input, const Trajectory & trajectory,
    const detail::SpatialIndex * obstacles) const;
  Config config_;
  std::vector<std::shared_ptr<const TrajectoryConstraint>> constraints_;
};
}  // namespace swerve_mppi
