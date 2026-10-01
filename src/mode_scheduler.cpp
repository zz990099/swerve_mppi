#include "swerve_mppi/mode.hpp"
#include "time_comparison.hpp"
#include "validation.hpp"
#include <algorithm>
#include <cmath>
namespace swerve_mppi {
ModeScheduler::ModeScheduler(const Config &config) : config_(config) { validate(config_); }
std::vector<Branch> ModeScheduler::make_branches(const VehicleState &state) const {
  if (!detail::valid_vehicle(state, config_))
    return {};
  std::vector<Branch> branches{{state.actual_mode, 0, false}};
  const double wait = config_.minimum_mode_dwell_s - state.time_in_mode_s;
  if (!state.mode_confirmed || state.mode_fault)
    return branches;
  const auto ticks = detail::duration_ticks(wait, config_.dt_s, config_.horizon_steps - 1);
  if (!ticks)
    return branches;
  const std::size_t earliest = *ticks;
  const DriveMode modes[] = {DriveMode::DualAckermann, DriveMode::Spin, DriveMode::Crab};
  for (DriveMode mode : modes) {
    if (mode == state.actual_mode)
      continue;
    if (earliest < config_.horizon_steps / 2) {
      branches.push_back({mode, earliest, true});
    }
    const std::size_t later = std::max(earliest, config_.horizon_steps / 3);
    if (later != earliest && later < config_.horizon_steps / 2) {
      branches.push_back({mode, later, true});
    }
  }
  return branches;
}

std::size_t ModeScheduler::select(const std::vector<Solution> &solutions) const {
  if (solutions.empty())
    return 0;
  std::size_t best = 0;
  for (std::size_t i = 1; i < solutions.size(); ++i) {
    const auto &candidate = solutions[i];
    if (!std::isfinite(candidate.cost))
      continue;
    if (candidate.branch.switches &&
        !(candidate.cost + config_.switch_hysteresis < solutions.front().cost))
      continue;
    if (candidate.cost < solutions[best].cost)
      best = i;
  }
  return best;
}
} // namespace swerve_mppi
