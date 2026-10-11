#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace swerve_mppi
{
using TimestampNs = std::int64_t;
constexpr TimestampNs kInvalidTimestamp = -1;
constexpr TimestampNs kNanosecondsPerSecond = 1000000000;

inline std::optional<TimestampNs> duration_nanoseconds(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0) {
    return std::nullopt;
  }
  const long double scaled =
    static_cast<long double>(seconds) * static_cast<long double>(kNanosecondsPerSecond);
  if (scaled > static_cast<long double>(std::numeric_limits<TimestampNs>::max())) {
    return std::nullopt;
  }
  return static_cast<TimestampNs>(std::llround(scaled));
}
inline double duration_seconds(TimestampNs nanoseconds)
{
  return static_cast<double>(nanoseconds) / kNanosecondsPerSecond;
}
inline std::optional<double> elapsed_seconds(TimestampNs later, TimestampNs earlier)
{
  if (later < 0 || earlier < 0 || later < earlier) {
    return std::nullopt;
  }
  return duration_seconds(later - earlier);
}
inline std::optional<TimestampNs> add_duration(TimestampNs stamp, double seconds)
{
  const auto duration = duration_nanoseconds(seconds);
  if (!duration || stamp < 0 || stamp > std::numeric_limits<TimestampNs>::max() - *duration) {
    return std::nullopt;
  }
  return stamp + *duration;
}
}  // namespace swerve_mppi
