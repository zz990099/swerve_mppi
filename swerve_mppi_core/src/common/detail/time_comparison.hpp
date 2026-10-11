#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

#include "swerve_mppi/common/time.hpp"

namespace swerve_mppi::detail
{
// Inclusive duration/deadline bounds. A nanosecond floor handles decimal tick
// arithmetic; four scaled machine epsilons cover subtraction of large stamps.
// Strictly increasing stamps and replay checks deliberately do not use this.
inline double time_tolerance(double a, double b)
{
  return std::max(
    1e-9, 4 * std::numeric_limits<double>::epsilon() * std::max(std::abs(a), std::abs(b)));
}
inline bool duration_exceeded(double elapsed, double limit)
{
  return elapsed - limit > time_tolerance(elapsed, limit);
}
inline bool deadline_exceeded(TimestampNs now, TimestampNs begin, double limit)
{
  const auto duration = duration_nanoseconds(limit);
  return !duration || now < begin || now - begin > *duration;
}
inline bool elapsed_at_least(TimestampNs now, TimestampNs begin, double minimum)
{
  const auto duration = duration_nanoseconds(minimum);
  return duration && now >= begin && now - begin >= *duration;
}
inline bool elapsed_at_least(double now, double begin, double minimum)
{
  return minimum - (now - begin) <=
         std::max(time_tolerance(now, begin), time_tolerance(minimum, minimum));
}
inline std::optional<std::size_t> duration_ticks(double duration, double dt, std::size_t maximum)
{
  if (duration_exceeded(duration, maximum * dt)) {
    return std::nullopt;
  }
  const double ticks = std::ceil(std::max(0.0, duration - time_tolerance(duration, dt)) / dt);
  if (
    !std::isfinite(ticks) ||
    ticks >= static_cast<double>(std::numeric_limits<std::size_t>::max())) {
    return std::nullopt;
  }
  return std::min(maximum, static_cast<std::size_t>(ticks));
}
}  // namespace swerve_mppi::detail
