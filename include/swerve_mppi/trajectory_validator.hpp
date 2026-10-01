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
  TrajectoryStatus check(const ControllerInput &input, const Trajectory &trajectory) const;

private:
  Config config_;
  std::vector<std::shared_ptr<const TrajectoryConstraint>> constraints_;
};
} // namespace swerve_mppi
