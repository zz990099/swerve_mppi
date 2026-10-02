#pragma once

#include "swerve_mppi/rollout.hpp"
#include <memory>

namespace swerve_mppi {
enum class TrajectoryStatus { Valid, Invalid, Collision, Rejected };

// Extra hard constraints, independent of optimization cost. Implementations
// must be const, bounded and safe to share across all controller policies.
class TrajectoryConstraint {
public:
  virtual ~TrajectoryConstraint() = default;
  virtual bool allows(const ControllerInput &input, const Trajectory &trajectory) const = 0;
};

class TrajectoryValidator {
public:
  explicit TrajectoryValidator(const Config &config);
  void add(std::shared_ptr<const TrajectoryConstraint> constraint);
  // Reject a different footprint, margin or measured joint validity envelope.
  // Planning-only parameters may differ because check() consumes explicit poses.
  void require_compatible(const Config &config) const;
  // The initial pose must match input.vehicle.pose within 1e-9 m/rad
  // (yaw modulo 2*pi). Also checks the exact current circular footprint.
  TrajectoryStatus check(const ControllerInput &input, const Trajectory &trajectory) const;

private:
  Config config_;
  std::vector<std::shared_ptr<const TrajectoryConstraint>> constraints_;
};
} // namespace swerve_mppi
