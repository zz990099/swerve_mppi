#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

namespace swerve_mppi
{
// Cooperative wall-clock budget, independent of ROS/simulation time. One shared
// instance covers all branches in a compute call. A clock hook supports exact
// boundary tests; production uses steady_clock. No hard real-time bound is
// implied.
class PlanningBudget
{
public:
  using Clock = std::chrono::steady_clock;
  using Now = std::function<Clock::time_point()>;
  explicit PlanningBudget(double seconds, Now now = {}) : seconds_(seconds), now_(std::move(now))
  {
    if (!std::isfinite(seconds) || seconds < 0) {
      throw std::invalid_argument("invalid planning budget");
    }
    if (seconds_ > 0) {
      start_ = current();
    }
  }
  bool expired() const
  {
    if (seconds_ == 0) {
      return false;  // Explicit offline/deterministic validation mode.
    }
    const auto now = current();
    // Round conservatively at the nanosecond boundary: dt * ratio may be
    // represented slightly above the exact deadline (for example .1 * .8).
    return now < start_ || std::chrono::duration<double>(now - start_).count() >= seconds_ - 1e-9;
  }

private:
  Clock::time_point current() const { return now_ ? now_() : Clock::now(); }
  double seconds_;
  Now now_;
  Clock::time_point start_{};
};
}  // namespace swerve_mppi
