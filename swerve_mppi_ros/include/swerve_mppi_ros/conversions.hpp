#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <swerve_gazebo_sim/msg/chassis_command.hpp>
#include <swerve_gazebo_sim/msg/chassis_state.hpp>

#include "swerve_mppi/adapter/current_chassis.hpp"

namespace swerve_mppi_ros
{
swerve_mppi::TimestampNs source_time(const builtin_interfaces::msg::Time & stamp);
builtin_interfaces::msg::Time ros_time(swerve_mppi::TimestampNs stamp);
swerve_mppi::Pose2d planar_pose(const geometry_msgs::msg::Pose & pose);
swerve_mppi::current_chassis::StateMessage convert(
  const swerve_gazebo_sim::msg::ChassisState & msg);
swerve_mppi::current_chassis::OdometryMessage convert(const nav_msgs::msg::Odometry & msg);
swerve_gazebo_sim::msg::ChassisCommand convert(
  const swerve_mppi::current_chassis::CommandMessage & msg);
}  // namespace swerve_mppi_ros
