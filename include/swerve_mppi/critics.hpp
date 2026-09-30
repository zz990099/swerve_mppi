#pragma once

#include "swerve_mppi/rollout.hpp"
#include <memory>
#include <string_view>

namespace swerve_mppi {
class Critic {
public:
  virtual ~Critic() = default;
  virtual std::string_view name() const = 0;
  virtual double score(const ControllerInput &input, const Trajectory &trajectory) const = 0;
};
class CriticManager {
public:
  explicit CriticManager(const Config &config);
  void add(std::shared_ptr<const Critic> critic);
  double score(const ControllerInput &input, const Trajectory &trajectory) const;

private:
  std::vector<std::shared_ptr<const Critic>> critics_;
};
} // namespace swerve_mppi
