#include <gtest/gtest.h>

#include <chrono>
#include <nav2_core/controller.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <swerve_gazebo_sim/msg/chassis_command.hpp>
#include <thread>

#include "swerve_mppi_ros/chassis_bridge.hpp"
#include "swerve_mppi_ros/conversions.hpp"
#include "swerve_mppi_ros/ordered_goal_checker.hpp"

using namespace std::chrono_literals;
using namespace swerve_mppi_ros;
using Observation = swerve_gazebo_sim::msg::ChassisState;
using Context = swerve_gazebo_sim::msg::ChassisCommand;
class Nav2PluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_controller_server");
    node->declare_parameter("FollowPath.samples", 16);
    node->declare_parameter("FollowPath.state_timeout", 0.5);
    node->declare_parameter("goal_checker.settle_time", 0.05);
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_local_costmap");
    costmap->set_parameters(
      {rclcpp::Parameter("global_frame", "odom"),
       rclcpp::Parameter("robot_base_frame", "base_footprint"),
       rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}),
       rclcpp::Parameter("track_unknown_space", false), rclcpp::Parameter("width", 8),
       rclcpp::Parameter("height", 8)});
    costmap->declare_parameter<std::string>(
      "inflation_layer.plugin", "nav2_costmap_2d::InflationLayer");
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    map_configured = true;
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    loader = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    goal_loader = std::make_unique<pluginlib::ClassLoader<nav2_core::GoalChecker>>(
      "nav2_core", "nav2_core::GoalChecker");
    controller = loader->createSharedInstance("swerve_mppi_ros::Nav2Controller");
    goal = goal_loader->createSharedInstance("swerve_mppi_ros::OrderedGoalChecker");
    controller->configure(node, "FollowPath", tf, costmap);
    goal->initialize(node, "goal_checker", costmap);
    controller->activate();
    active = true;
    source = std::make_shared<rclcpp::Node>("test_chassis_source");
    bridge = std::make_shared<ChassisBridge>();
    states =
      source->create_publisher<Observation>("chassis_controller/state", rclcpp::SensorDataQoS());
    velocities = source->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    contexts = source->create_subscription<Context>(
      "mppi/command_context", 10, [&](const Context & c) { context = c; });
    commands = source->create_subscription<Context>(
      "chassis_controller/command", 10, [&](const Context & c) { command = c; });
    executor->add_node(node->get_node_base_interface());
    executor->add_node(source);
    executor->add_node(bridge);
    for (int i = 0; i < 100 && (states->get_subscription_count() < 2 ||
                                velocities->get_subscription_count() < 1);
         ++i)
      spin(10);
    observed.header.frame_id = "base_footprint";
    observed.confirmed = true;
    observed.phase = Observation::READY;
    publish_state();
  }
  void TearDown() override
  {
    if (controller) {
      if (active) controller->deactivate();
      controller->cleanup();
    }
    if (executor && source && bridge) {
      executor->remove_node(node->get_node_base_interface());
      executor->remove_node(source);
      executor->remove_node(bridge);
    }
    executor.reset();
    controller.reset();
    goal.reset();
    loader.reset();
    goal_loader.reset();
    bridge.reset();
    source.reset();
    node.reset();
    if (costmap && map_configured) costmap->on_cleanup(rclcpp_lifecycle::State());
    costmap.reset();
    tf.reset();
    rclcpp::shutdown();
  }
  void spin(int ms)
  {
    for (int i = 0; i < ms; ++i) {
      executor->spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }
  void publish_state()
  {
    observed.header.stamp = source->now();
    states->publish(observed);
    spin(20);
  }
  nav_msgs::msg::Path path(std::vector<Pose> points)
  {
    nav_msgs::msg::Path out;
    out.header.frame_id = "odom";
    for (auto p : points) {
      geometry_msgs::msg::PoseStamped item;
      item.header = out.header;
      item.pose = pose_message(p);
      out.poses.push_back(item);
    }
    return out;
  }
  geometry_msgs::msg::TwistStamped compute(Pose p = {})
  {
    publish_state();
    geometry_msgs::msg::PoseStamped current;
    current.header.stamp = source->now();
    current.header.frame_id = "odom";
    current.pose = pose_message(p);
    return controller->computeVelocityCommands(current, geometry_msgs::msg::Twist{}, goal.get());
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<rclcpp::Node> source;
  std::shared_ptr<ChassisBridge> bridge;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> goal_loader;
  nav2_core::Controller::Ptr controller;
  nav2_core::GoalChecker::Ptr goal;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<Observation>::SharedPtr states;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocities;
  rclcpp::Subscription<Context>::SharedPtr contexts, commands;
  bool active = false, map_configured = false;
  Observation observed;
  Context context, command;
};
TEST_F(Nav2PluginTest, LifecycleSpeedLimitsAndServerVelocityAuthority)
{
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  auto speed = compute();
  EXPECT_GT(speed.twist.linear.x, 0);
  EXPECT_EQ(speed.header.frame_id, "base_footprint");
  spin(20);
  EXPECT_EQ(command.velocity.linear.x, 0) << "Mode context alone cannot move the chassis";
  velocities->publish(speed);
  spin(20);
  EXPECT_GT(command.velocity.linear.x, 0);
  auto zero = speed;
  zero.header.stamp = source->now();
  zero.twist = geometry_msgs::msg::Twist{};
  velocities->publish(zero);
  spin(20);
  EXPECT_EQ(command.velocity.linear.x, 0) << "Nav2 zero must override a nonzero controller context";
  controller->setSpeedLimit(0.1, false);
  speed = compute();
  EXPECT_LE(std::hypot(speed.twist.linear.x, speed.twist.linear.y), 0.1 + 1e-9);
  controller->setSpeedLimit(25.0, true);
  speed = compute();
  EXPECT_LE(std::hypot(speed.twist.linear.x, speed.twist.linear.y), 0.2 + 1e-9);
  controller->setSpeedLimit(0.0, false);  // Nav2's NO_SPEED_LIMIT sentinel.
  EXPECT_GT(compute().twist.linear.x, 0.1);
  EXPECT_TRUE(controller->cancel());
  spin(20);
  EXPECT_EQ(command.velocity.linear.x, 0);
  EXPECT_THROW(compute(), nav2_core::InvalidPath);
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  EXPECT_GT(compute().twist.linear.x, 0);
  controller->deactivate();
  EXPECT_THROW(compute(), nav2_core::ControllerException);
  controller->activate();
}
TEST_F(Nav2PluginTest, ModeHandshakeAndOrderedGoalChecker)
{
  controller->setPlan(path({{0, 0, 0}, {0, 1, 0}}));
  auto speed = compute();
  spin(20);
  EXPECT_EQ(context.mode, Observation::CRAB);
  EXPECT_EQ(context.request_id, 1u);
  EXPECT_EQ(speed.twist.linear.y, 0);
  const auto entry = context.entry_velocity;
  // No chassis echo yet: the plugin keeps its pending request and entry frozen.
  compute();
  spin(20);
  EXPECT_EQ(context.request_id, 1u);
  EXPECT_EQ(context.entry_velocity, entry);
  observed.request_id = 1;
  observed.actual_mode = observed.requested_mode = Observation::CRAB;
  observed.accepted_entry_velocity = entry;
  for (auto & a : observed.steering_angles) a = std::acos(-1.0) / 2;
  compute();
  observed = Observation{};
  observed.header.frame_id = "base_footprint";
  observed.confirmed = true;
  observed.phase = Observation::READY;
  publish_state();
  controller->reset();
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}, {0, 0, 0}}));
  goal->reset();
  const auto origin = pose_message({});
  EXPECT_FALSE(goal->isGoalReached(
    origin, origin, geometry_msgs::msg::Twist{}));  // Nav2 checks before first compute.
  compute();
  std::this_thread::sleep_for(60ms);
  EXPECT_FALSE(goal->isGoalReached(origin, origin, geometry_msgs::msg::Twist{}));
  compute({1, 0, 0});
  compute({0, 0, 0});
  EXPECT_FALSE(goal->isGoalReached(origin, origin, geometry_msgs::msg::Twist{}));
  std::this_thread::sleep_for(60ms);
  EXPECT_TRUE(goal->isGoalReached(origin, origin, geometry_msgs::msg::Twist{}));
}
TEST_F(Nav2PluginTest, CostmapTfAndInvalidInputs)
{
  EXPECT_THROW(controller->setPlan(path({})), nav2_core::InvalidPath);
  auto mixed = path({{0, 0, 0}, {1, 0, 0}});
  mixed.poses.back().header.frame_id = "map";
  EXPECT_THROW(controller->setPlan(mixed), nav2_core::InvalidPath);
  auto missing_tf = path({{0, 0, 0}, {1, 0, 0}});
  missing_tf.header.frame_id = "missing";
  for (auto & p : missing_tf.poses) p.header = missing_tf.header;
  controller->setPlan(missing_tf);
  EXPECT_THROW(compute(), nav2_core::ControllerTFError);
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  auto map = costmap->getCostmap();
  unsigned int x, y;
  ASSERT_TRUE(map->worldToMap(0, 0, x, y));
  map->setCost(x, y, 254);
  EXPECT_THROW(compute(), nav2_core::NoValidControl);
  map->setCost(x, y, 0);
  EXPECT_GT(compute().twist.linear.x, 0);
  spin(550);
  geometry_msgs::msg::PoseStamped current;
  current.header.frame_id = "odom";
  current.pose = pose_message({});
  EXPECT_THROW(
    controller->computeVelocityCommands(current, geometry_msgs::msg::Twist{}, goal.get()),
    nav2_core::NoValidControl);
}
TEST_F(Nav2PluginTest, BridgeRejectsStaleAndModeIncompatibleSpeed)
{
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  auto speed = compute();
  spin(20);
  auto stale = speed;
  stale.header.stamp = source->now() - rclcpp::Duration::from_seconds(1);
  velocities->publish(stale);
  spin(20);
  EXPECT_EQ(command.velocity.linear.x, 0);
  auto wrong = speed;
  wrong.header.stamp = source->now();
  wrong.twist.linear.y = 0.2;
  velocities->publish(wrong);
  spin(20);
  EXPECT_EQ(command.velocity.linear.x, 0);
  EXPECT_EQ(command.velocity.linear.y, 0);
  speed.header.stamp = source->now();
  velocities->publish(speed);
  spin(20);
  EXPECT_GT(command.velocity.linear.x, 0);
  // Fresh chassis feedback cannot renew an expired plugin context.
  spin(280);
  publish_state();
  speed.header.stamp = source->now();
  velocities->publish(speed);
  spin(20);
  EXPECT_EQ(command.velocity.linear.x, 0);
}
TEST_F(Nav2PluginTest, PluginDoesNotRetainItsLifecycleParent)
{
  auto separate = loader->createSharedInstance("swerve_mppi_ros::Nav2Controller");
  auto owner = std::make_shared<rclcpp_lifecycle::LifecycleNode>("temporary_controller_parent");
  auto buffer = std::make_shared<tf2_ros::Buffer>(owner->get_clock());
  separate->configure(owner, "FollowPath", buffer, costmap);
  rclcpp_lifecycle::LifecycleNode::WeakPtr weak = owner;
  owner.reset();
  EXPECT_TRUE(weak.expired()) << "A controller plugin must not create a server ownership cycle";
  separate->cleanup();
}
