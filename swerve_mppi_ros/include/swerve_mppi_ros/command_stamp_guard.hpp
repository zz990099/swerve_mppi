#pragma once

#include <algorithm>
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
    // The chassis compares timestamps as double seconds, including in wall-clock mode.
    constexpr std::int64_t stamp_step = 1'000;
    if (now <= 0 || source <= 0 || source - now > future_tolerance || source < last_source_)
      return std::nullopt;
    if (source == last_source_ && (!stopped || last_stopped_)) return std::nullopt;

    const auto latest_output = now + future_tolerance;
    const auto next_output = last_output_ == 0 ? 1 : last_output_ + stamp_step;
    // Leave room for a strictly newer stop even if /clock stays frozen.
    const auto latest_motion = latest_output - stamp_step;
    const auto output = stopped ? std::max(source, next_output) : std::min(source, latest_motion);
    if (output < next_output || output > latest_output) return std::nullopt;
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
