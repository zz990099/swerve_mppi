#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <stdexcept>
#include <swerve_gazebo_sim/msg/chassis_state.hpp>

#include "swerve_mppi_ros/control_support.hpp"

namespace swerve_mppi_ros
{
inline double seconds(const builtin_interfaces::msg::Time & t) { return t.sec + t.nanosec * 1e-9; }
inline bool finite(Velocity v)
{
  return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.wz);
}
inline Velocity velocity(const geometry_msgs::msg::Twist & v)
{
  if (v.linear.z != 0 || v.angular.x != 0 || v.angular.y != 0)
    throw std::invalid_argument("non-planar velocity");
  Velocity out{v.linear.x, v.linear.y, v.angular.z};
  if (!finite(out)) throw std::invalid_argument("non-finite velocity");
  return out;
}
inline geometry_msgs::msg::Twist twist(Velocity v)
{
  geometry_msgs::msg::Twist out;
  out.linear.x = v.vx;
  out.linear.y = v.vy;
  out.angular.z = v.wz;
  return out;
}
inline Pose pose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (
    !std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(norm) ||
    std::abs(norm - 1) > 0.01 || std::abs(q.x) > 0.01 || std::abs(q.y) > 0.01)
    throw std::invalid_argument("invalid planar pose");
  return {p.position.x, p.position.y, std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)};
}
inline geometry_msgs::msg::Pose pose_message(Pose p)
{
  geometry_msgs::msg::Pose out;
  out.position.x = p.x;
  out.position.y = p.y;
  out.orientation.z = std::sin(p.yaw / 2);
  out.orientation.w = std::cos(p.yaw / 2);
  return out;
}
inline ChassisFeedback feedback(const swerve_gazebo_sim::msg::ChassisState & msg, double radius)
{
  if (msg.actual_mode > 2 || msg.requested_mode > 2)
    throw std::invalid_argument("invalid chassis mode");
  ChassisFeedback f;
  f.request_id = msg.request_id;
  f.entry = velocity(msg.accepted_entry_velocity);
  f.fault = msg.fault != 0;
  f.state.mode = static_cast<Mode>(msg.actual_mode);
  f.state.requested_mode = static_cast<Mode>(msg.requested_mode);
  f.state.switching = !msg.confirmed;
  f.state.steering = msg.steering_angles;
  for (std::size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(msg.steering_angles[i]) || !std::isfinite(msg.wheel_speeds[i]))
      throw std::invalid_argument("invalid joint feedback");
    f.state.wheel_speeds[i] = msg.wheel_speeds[i] * radius;
  }
  return f;
}
inline bool permitted(Mode mode, Velocity v)
{
  if (!valid_mode(mode) || !finite(v)) return false;
  if (mode == Mode::Spin) return v.vx == 0 && v.vy == 0;
  if (mode == Mode::Crab) return v.wz == 0;
  return v.vy == 0 && (v.vx != 0 || v.wz == 0);
}
}  // namespace swerve_mppi_ros
