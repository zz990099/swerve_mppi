#pragma once

#include <map>
#include <optional>
#include <utility>

#include "swerve_mppi_ros/conversions.hpp"

namespace swerve_mppi_ros
{
// Bounded asynchronous pairing. DDS duplicates never become new observations.
class ObservationBuffer
{
public:
  using Pair = std::pair<swerve_gazebo_sim::msg::ChassisState, nav_msgs::msg::Odometry>;
  void push(const swerve_gazebo_sim::msg::ChassisState & msg);
  void push(const nav_msgs::msg::Odometry & msg);
  std::optional<Pair> take(swerve_mppi::TimestampNs tolerance_ns);

private:
  std::map<swerve_mppi::TimestampNs, swerve_gazebo_sim::msg::ChassisState> states_;
  std::map<swerve_mppi::TimestampNs, nav_msgs::msg::Odometry> odometry_;
  std::optional<Pair> consumed_;
};
}  // namespace swerve_mppi_ros
