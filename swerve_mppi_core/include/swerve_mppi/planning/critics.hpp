#pragma once

#include <memory>
#include <string_view>

#include "swerve_mppi/safety/trajectory_validator.hpp"

namespace swerve_mppi
{
namespace detail
{
struct ScoringGeometry;
}
class Critic
{
public:
  virtual ~Critic() = default;
  virtual std::string_view name() const = 0;
  virtual double score(const ControllerInput & input, const Trajectory & trajectory) const = 0;
};
class CriticManager
{
public:
  explicit CriticManager(
    const Config & config, std::shared_ptr<const TrajectoryValidator> validator = nullptr);
  void add(std::shared_ptr<const Critic> critic);
  double score(const ControllerInput & input, const Trajectory & trajectory) const;

private:
  friend class Optimizer;
  bool prepare(const ControllerInput & input);
  double prepared_curvature() const;
  double score_prepared(const ControllerInput & input, const Trajectory & trajectory) const;
  std::shared_ptr<const detail::ScoringGeometry> geometry_;
  Config config_;
  std::shared_ptr<const TrajectoryValidator> validator_;
  std::vector<std::shared_ptr<const Critic>> critics_;
};
}  // namespace swerve_mppi
