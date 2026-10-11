#include "swerve_mppi_ros/conversions.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_mppi_ros
{
namespace
{
swerve_mppi::current_chassis::TwistMessage twist(const geometry_msgs::msg::Twist & msg)
{
  return {msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z};
}
geometry_msgs::msg::Twist twist(const swerve_mppi::current_chassis::TwistMessage & msg)
{
  geometry_msgs::msg::Twist out;
  out.linear.x = msg.linear_x;
  out.linear.y = msg.linear_y;
  out.linear.z = msg.linear_z;
  out.angular.x = msg.angular_x;
  out.angular.y = msg.angular_y;
  out.angular.z = msg.angular_z;
  return out;
}
}  // namespace

swerve_mppi::TimestampNs source_time(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {
    throw std::invalid_argument("invalid source timestamp");
  }
  return static_cast<swerve_mppi::TimestampNs>(stamp.sec) * 1000000000 + stamp.nanosec;
}

builtin_interfaces::msg::Time ros_time(swerve_mppi::TimestampNs stamp)
{
  if (stamp < 0 || stamp / 1000000000 > std::numeric_limits<int32_t>::max()) {
    throw std::invalid_argument("timestamp outside ROS Time range");
  }
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<int32_t>(stamp / 1000000000);
  out.nanosec = static_cast<uint32_t>(stamp % 1000000000);
  return out;
}

swerve_mppi::Pose2d planar_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (
    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) ||
    std::abs(norm - 1) > 1e-6 || std::abs(p.z) > 1e-9 || std::abs(q.x) > 1e-9 ||
    std::abs(q.y) > 1e-9) {
    throw std::invalid_argument("pose must be finite, planar and have a unit quaternion");
  }
  return {p.x, p.y, std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)};
}

swerve_mppi::current_chassis::StateMessage convert(const swerve_gazebo_sim::msg::ChassisState & msg)
{
  swerve_mppi::current_chassis::StateMessage out;
  out.stamp_ns = source_time(msg.header.stamp);
  out.frame_id = msg.header.frame_id;
  out.request_id = msg.request_id;
  out.actual_mode = msg.actual_mode;
  out.requested_mode = msg.requested_mode;
  out.phase = msg.phase;
  out.confirmed = msg.confirmed;
  out.fault = msg.fault;
  out.velocity = twist(msg.velocity);
  out.steering_angles = msg.steering_angles;
  out.wheel_speeds_radps = msg.wheel_speeds;
  out.accepted_entry_velocity = twist(msg.accepted_entry_velocity);
  out.accepted_steering = msg.accepted_steering;
  return out;
}

swerve_mppi::current_chassis::OdometryMessage convert(const nav_msgs::msg::Odometry & msg)
{
  return {
    source_time(msg.header.stamp), msg.header.frame_id, msg.child_frame_id,
    planar_pose(msg.pose.pose), twist(msg.twist.twist)};
}

swerve_gazebo_sim::msg::ChassisCommand convert(
  const swerve_mppi::current_chassis::CommandMessage & msg)
{
  swerve_gazebo_sim::msg::ChassisCommand out;
  out.header.stamp = ros_time(msg.stamp_ns);
  out.header.frame_id = msg.frame_id;
  out.mode = msg.mode;
  out.velocity = twist(msg.velocity);
  out.request_id = msg.request_id;
  out.entry_velocity = twist(msg.entry_velocity);
  return out;
}
}  // namespace swerve_mppi_ros
