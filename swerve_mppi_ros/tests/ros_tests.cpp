#include <gtest/gtest.h>

#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <swerve_gazebo_sim/msg/chassis_command.hpp>
#include <swerve_gazebo_sim/msg/chassis_state.hpp>
#include <thread>

#include "swerve_mppi_ros/controller_node.hpp"

using namespace std::chrono_literals;
TEST(RosCaller, ReplansWithoutReceiptsAndGoalCheckerLivesOutsideCore)
{
  rclcpp::init(0, nullptr);
  auto source = std::make_shared<rclcpp::Node>("synthetic_chassis");
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("samples", 16)});
  auto controller = std::make_shared<swerve_mppi_ros::ControllerNode>(options);
  auto state_pub = source->create_publisher<swerve_gazebo_sim::msg::ChassisState>(
    "chassis_controller/state", rclcpp::SensorDataQoS());
  auto odom_pub = source->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  auto path_pub = source->create_publisher<nav_msgs::msg::Path>("path", 1);
  std::size_t commands = 0;
  swerve_gazebo_sim::msg::ChassisCommand accepted;
  bool moving = false, reached = false;
  auto command_sub = source->create_subscription<swerve_gazebo_sim::msg::ChassisCommand>(
    "chassis_controller/command", 100, [&](const swerve_gazebo_sim::msg::ChassisCommand & m) {
      ++commands;
      accepted = m;
      moving = moving || m.velocity.linear.x > 0;
    });
  auto goal_sub = source->create_subscription<std_msgs::msg::Bool>(
    "/mppi_controller/goal_reached", rclcpp::QoS(1).transient_local(),
    [&](const std_msgs::msg::Bool & m) { reached = m.data; });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(source);
  executor.add_node(controller);
  auto spin = [&](int cycles, bool endpoint) {
    for (int i = 0; i < cycles; ++i) {
      swerve_gazebo_sim::msg::ChassisState state;
      state.header.stamp = source->now();
      state.header.frame_id = "base_footprint";
      state.request_id = accepted.request_id;
      state.actual_mode = state.requested_mode = accepted.mode;
      state.accepted_entry_velocity = accepted.entry_velocity;
      state.confirmed = true;
      state.phase = state.READY;
      state_pub->publish(state);
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = state.header.stamp;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";
      odom.pose.pose.orientation.w = 1;
      odom.pose.pose.position.x = endpoint ? 1 : 0;
      odom_pub->publish(odom);
      executor.spin_some();
      std::this_thread::sleep_for(20ms);
    }
  };
  spin(25, false);
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped p;
  p.header = path.header;
  p.pose.orientation.w = 1;
  path.poses.push_back(p);
  p.pose.position.x = 1;
  path.poses.push_back(p);
  ASSERT_GT(path_pub->get_subscription_count(), 0u);
  const auto before_task = commands;
  path_pub->publish(path);
  spin(50, false);
  EXPECT_GT(commands - before_task, 5u);
  EXPECT_TRUE(moving);
  EXPECT_FALSE(reached);
  spin(30, true);
  EXPECT_TRUE(reached);
  // A new task clears external completion and core warm-start state.
  path_pub->publish(path);
  spin(15, false);
  EXPECT_FALSE(reached);
  executor.remove_node(controller);
  executor.remove_node(source);
  controller.reset();
  source.reset();
  rclcpp::shutdown();
}
