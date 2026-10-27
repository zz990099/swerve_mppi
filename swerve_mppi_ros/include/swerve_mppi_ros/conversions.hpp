#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <stdexcept>
#include <swerve_chassis_msgs/msg/chassis_state.hpp>

#include "swerve_mppi/model.hpp"
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
inline ChassisFeedback feedback(
  const swerve_chassis_msgs::msg::ChassisState & msg, const Config & c)
{
  auto equal = [](double a, double b) {
    return std::isfinite(a) && std::isfinite(b) &&
           std::abs(a - b) <= 1e-9 * std::max({1.0, std::abs(a), std::abs(b)});
  };
  const auto & l = c.limits;
  std::array<double, 3> geometry{c.geometry.wheelbase, c.geometry.track_width, c.geometry.radius};
  std::array<double, 10> limits{
    l.steering,           l.steering_rate, l.wheel_speed,         l.wheel_acceleration,
    l.linear_speed,       l.angular_speed, l.linear_acceleration, l.angular_acceleration,
    l.tracking_tolerance, l.stopped_speed};
  for (std::size_t i = 0; i < 3; ++i)
    if (!equal(msg.geometry[i], geometry[i]))
      throw std::invalid_argument("chassis geometry mismatch");
  for (std::size_t i = 0; i < 10; ++i)
    if (!equal(msg.limits[i], limits[i])) throw std::invalid_argument("chassis limits mismatch");
  if (
    msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000 ||
    msg.feedback_stamp.sec < 0 || msg.feedback_stamp.nanosec >= 1000000000 || msg.execution > 4 ||
    msg.fault > 3)
    throw std::invalid_argument("invalid chassis metadata");
  ChassisFeedback f;
  f.fault = msg.fault != 0;
  f.feedback_valid = msg.feedback_valid;
  f.feedback_stamp = seconds(msg.feedback_stamp);
  f.state.steering = msg.steering_angles;
  f.state.wheel_speeds = msg.wheel_speeds;
  f.state.velocity = velocity(msg.velocity);
  f.state.generator = {{msg.target_steering, msg.target_wheel_speeds}, msg.generator_valid};
  if (!f.fault && !Model(c).valid(f.state))
    throw std::invalid_argument("invalid execution snapshot");
  return f;
}
}  // namespace swerve_mppi_ros
