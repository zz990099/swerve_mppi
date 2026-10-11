#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "swerve_mppi_ros/observation_buffer.hpp"

namespace
{
using namespace swerve_mppi_ros;

swerve_gazebo_sim::msg::ChassisState state(int64_t stamp)
{
  swerve_gazebo_sim::msg::ChassisState msg;
  msg.header.stamp = ros_time(stamp);
  msg.header.frame_id = "base_footprint";
  msg.phase = msg.READY;
  msg.confirmed = true;
  return msg;
}
nav_msgs::msg::Odometry odom(int64_t stamp)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = ros_time(stamp);
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_footprint";
  msg.pose.pose.orientation.w = 1;
  return msg;
}

TEST(Transport, TimeAndPlanarPoseRejectInvalidRepresentations)
{
  EXPECT_EQ(source_time(ros_time(1234567890123456)), 1234567890123456);
  EXPECT_THROW(ros_time(-1), std::invalid_argument);
  builtin_interfaces::msg::Time bad;
  bad.nanosec = 1000000000;
  EXPECT_THROW(source_time(bad), std::invalid_argument);
  bad.sec = -1;
  bad.nanosec = 0;
  EXPECT_THROW(source_time(bad), std::invalid_argument);
  auto pose = odom(1).pose.pose;
  pose.orientation.z = std::sin(0.3);
  pose.orientation.w = std::cos(0.3);
  EXPECT_NEAR(planar_pose(pose).yaw, 0.6, 1e-12);
  pose.position.z = 0.1;
  EXPECT_THROW(planar_pose(pose), std::invalid_argument);
  pose.position.z = 0;
  pose.orientation.x = 0.1;
  EXPECT_THROW(planar_pose(pose), std::invalid_argument);
  pose.orientation = geometry_msgs::msg::Quaternion();
  pose.orientation.w = 0;
  EXPECT_THROW(planar_pose(pose), std::invalid_argument);
}

TEST(Transport, CurrentMessageFieldsAndAllModesArePreserved)
{
  for (uint8_t mode = 0; mode < 3; ++mode) {
    auto msg = state(42);
    msg.actual_mode = msg.requested_mode = mode;
    msg.request_id = 25;
    msg.wheel_speeds = {1, 2, 3, 4};
    msg.accepted_entry_velocity.linear.y = 0.4;
    msg.accepted_steering = {0.1, 0.2, 0.3, 0.4};
    const auto dto = convert(msg);
    EXPECT_EQ(dto.stamp_ns, 42);
    EXPECT_EQ(dto.actual_mode, mode);
    EXPECT_EQ(dto.request_id, 25u);
    EXPECT_EQ(dto.wheel_speeds_radps, msg.wheel_speeds);
    EXPECT_EQ(dto.accepted_steering, msg.accepted_steering);
    EXPECT_EQ(dto.accepted_entry_velocity.linear_y, 0.4);
    swerve_mppi::current_chassis::CommandMessage command;
    command.stamp_ns = 45;
    command.frame_id = "base_footprint";
    command.mode = mode;
    command.request_id = 25;
    command.entry_velocity = dto.accepted_entry_velocity;
    const auto wire = convert(command);
    EXPECT_EQ(wire.mode, mode);
    EXPECT_EQ(wire.request_id, 25u);
    EXPECT_EQ(wire.entry_velocity.linear.y, 0.4);
    EXPECT_EQ(source_time(wire.header.stamp), 45);
  }
}

TEST(Transport, SourcePairingHandlesOrderDuplicatesAndBoundedRetention)
{
  ObservationBuffer buffer;
  buffer.push(state(100));
  buffer.push(odom(107));
  EXPECT_FALSE(buffer.take(5));
  buffer.push(odom(102));
  const auto pair = buffer.take(5);
  ASSERT_TRUE(pair);
  EXPECT_EQ(source_time(pair->second.header.stamp), 102);
  buffer.push(state(100));
  buffer.push(odom(102));
  EXPECT_FALSE(buffer.take(5));
  auto conflicting = state(100);
  conflicting.request_id = 1;
  EXPECT_THROW(buffer.push(conflicting), std::invalid_argument);
  // Old delayed DDS samples are dropped, never relabelled as fresh.
  buffer.push(state(99));
  buffer.push(odom(99));
  EXPECT_FALSE(buffer.take(5));
  for (int64_t i = 200; i < 240; ++i) buffer.push(state(i));
  buffer.push(odom(200));
  EXPECT_FALSE(buffer.take(0));
  buffer.push(odom(239));
  ASSERT_TRUE(buffer.take(0));
  EXPECT_FALSE(buffer.take(5));
}
}  // namespace
