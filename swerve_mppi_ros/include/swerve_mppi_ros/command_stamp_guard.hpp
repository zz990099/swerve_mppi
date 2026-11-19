#pragma once

#include <cstdint>
#include <optional>

namespace swerve_mppi_ros
{
class CommandStampGuard
{
public:
  std::optional<std::int64_t> accept(std::int64_t source, std::int64_t now, bool stopped)
  {
    if (now < last_clock_) {
      last_source_ = last_output_ = 0;
      last_stopped_ = false;
    }
    last_clock_ = now;
    constexpr std::int64_t future_tolerance = 30'000'000;
    if (now <= 0 || source <= 0 || source - now > future_tolerance || source < last_source_)
      return std::nullopt;
    auto output = source;
    if (source == last_source_) {
      if (!stopped || last_stopped_) return std::nullopt;
      // Publish the stop immediately even if /clock has not ticked. One
      // microsecond is distinguishable by the chassis' double-second stamps.
      output += 1000;
    }
    if (output <= last_output_ || output - now > future_tolerance) return std::nullopt;
    last_source_ = source;
    last_output_ = output;
    last_stopped_ = stopped;
    return output;
  }

private:
  std::int64_t last_clock_ = 0;
  std::int64_t last_source_ = 0;
  std::int64_t last_output_ = 0;
  bool last_stopped_ = false;
};
}  // namespace swerve_mppi_ros
