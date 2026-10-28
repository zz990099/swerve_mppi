#include <gtest/gtest.h>

#include <chrono>
#include <nav2_core/controller.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <thread>

#include "swerve_mppi_ros/conversions.hpp"
#include "swerve_mppi_ros/ordered_goal_checker.hpp"

using namespace std::chrono_literals;
using namespace swerve_mppi_ros;
using Observation = swerve_chassis_msgs::msg::ChassisState;
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
    costmap->getCostmap()->updateOrigin(-4.0, -4.0);
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
    states =
      source->create_publisher<Observation>("chassis_controller/state", rclcpp::SensorDataQoS());
    executor->add_node(node->get_node_base_interface());
    executor->add_node(source);
    for (int i = 0; i < 100 && states->get_subscription_count() < 1; ++i) spin(10);
    observed.header.frame_id = "base_footprint";
    observed.generator_valid = true;
    observed.feedback_valid = true;
    observed.geometry = {0.6, 0.5, 0.1};
    observed.limits = {swerve_chassis_core::pi / 2, 2.5, 20, 40, 0.8, 0.8, 0.9, 1.3, 0.05, 0.05};
    publish_state();
  }
  void TearDown() override
  {
    if (controller) {
      if (active) controller->deactivate();
      controller->cleanup();
    }
    if (executor && source) {
      executor->remove_node(node->get_node_base_interface());
      executor->remove_node(source);
    }
    executor.reset();
    controller.reset();
    goal.reset();
    loader.reset();
    goal_loader.reset();
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
    observed.feedback_stamp = observed.header.stamp;
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
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> goal_loader;
  nav2_core::Controller::Ptr controller;
  nav2_core::GoalChecker::Ptr goal;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<Observation>::SharedPtr states;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocities;
  bool active = false, map_configured = false;
  Observation observed;
};
TEST_F(Nav2PluginTest, LifecycleSpeedLimitsAndServerVelocityAuthority)
{
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  auto speed = compute();
  EXPECT_GT(speed.twist.linear.x, 0);
  EXPECT_EQ(speed.header.frame_id, "base_footprint");
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
  EXPECT_THROW(compute(), nav2_core::InvalidPath);
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  EXPECT_GT(compute().twist.linear.x, 0);
  controller->deactivate();
  EXPECT_THROW(compute(), nav2_core::ControllerException);
  controller->activate();
}
TEST_F(Nav2PluginTest, ContinuousVelocityAndOrderedGoalChecker)
{
  controller->setPlan(path({{0, 0, 0}, {0.5, 0.5, 0.5}}));
  const auto speed = compute();
  EXPECT_GT(speed.twist.linear.y, 0);
  EXPECT_GT(speed.twist.angular.z, 0);
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
  observed.velocity.linear.z = 1;  // Invalid feedback must clear an earlier stopped sample.
  publish_state();
  EXPECT_FALSE(goal->isGoalReached(origin, origin, geometry_msgs::msg::Twist{}));
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
TEST_F(Nav2PluginTest, FaultRecoveryAndConfigurationMismatch)
{
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  observed.fault = 1;
  observed.generator_valid = false;
  EXPECT_EQ(compute().twist.linear.x, 0);
  observed.fault = 0;
  observed.generator_valid = true;
  EXPECT_GT(compute().twist.linear.x, 0);
  observed.geometry[0] = 0.7;
  EXPECT_THROW(compute(), nav2_core::NoValidControl);
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
