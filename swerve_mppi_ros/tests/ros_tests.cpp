#include <gtest/gtest.h>

#include <chrono>
#include <nav2_core/controller.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rcl/time.h>
#include <thread>

#include "swerve_mppi_ros/conversions.hpp"
#include "swerve_mppi_ros/ordered_goal_checker.hpp"

using namespace std::chrono_literals;
using namespace swerve_mppi_ros;
using Observation = swerve_chassis_msgs::msg::ChassisState;
class GoalCheckerWithoutTolerances : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override
  {
    return false;
  }
  bool getTolerances(geometry_msgs::msg::Pose &, geometry_msgs::msg::Twist &) override
  {
    return false;
  }
};
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
    node->declare_parameter("FollowPath.state_future_tolerance", 0.03);
    node->declare_parameter("goal_checker.settle_time", 0.05);
    node->declare_parameter("goal_checker.state_future_tolerance", 0.03);
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
    auto result =
      controller->computeVelocityCommands(current, geometry_msgs::msg::Twist{}, goal.get());
    observed.requested_velocity = result.twist;
    return result;
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
  EXPECT_GT(compute().twist.linear.x, 0.0);
  EXPECT_TRUE(controller->cancel());
  spin(20);
  EXPECT_THROW(compute(), nav2_core::InvalidPath);
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  EXPECT_GT(compute().twist.linear.x, 0);
  controller->deactivate();
  EXPECT_THROW(compute(), nav2_core::ControllerException);
  controller->activate();
}
TEST_F(Nav2PluginTest, RequestSlewUsesPublishedOutputWhenFeedbackLags)
{
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  geometry_msgs::msg::TwistStamped previous;
  for (int i = 0; i < 5; ++i) {
    // Re-publish a fresh actuator snapshot carrying an old accepted request.
    observed.requested_velocity = geometry_msgs::msg::Twist{};
    if (i) observed.requested_velocity.linear.x = -0.2;
    const auto current = compute();
    if (i) {
      const double dt = seconds(current.header.stamp) - seconds(previous.header.stamp);
      EXPECT_LE(
        std::hypot(
          current.twist.linear.x - previous.twist.linear.x,
          current.twist.linear.y - previous.twist.linear.y),
        dt + 1e-7);
      EXPECT_LE(std::abs(current.twist.angular.z - previous.twist.angular.z), 1.5 * dt + 1e-7);
    }
    previous = current;
  }
}
TEST_F(Nav2PluginTest, FutureSnapshotDoesNotEraseFreshValidatedFeedback)
{
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  compute();
  observed.header.stamp = source->now() + rclcpp::Duration::from_seconds(10.);
  observed.feedback_stamp = observed.header.stamp;
  states->publish(observed);
  spin(20);
  geometry_msgs::msg::PoseStamped current;
  current.header.frame_id = "odom";
  current.pose = pose_message({});
  EXPECT_NO_THROW(
    controller->computeVelocityCommands(current, geometry_msgs::msg::Twist{}, goal.get()));
  // Ignoring a future sample cannot refresh the age of the retained snapshot.
  spin(550);
  EXPECT_THROW(
    controller->computeVelocityCommands(current, geometry_msgs::msg::Twist{}, goal.get()),
    nav2_core::NoValidControl);
  compute();  // Restore a fresh snapshot, then deliver a future-dated fault.
  observed.header.stamp = source->now() + rclcpp::Duration::from_seconds(10.);
  observed.feedback_stamp = observed.header.stamp;
  observed.fault = 1;
  states->publish(observed);
  spin(20);
  EXPECT_THROW(
    controller->computeVelocityCommands(current, geometry_msgs::msg::Twist{}, goal.get()),
    nav2_core::NoValidControl);
}
TEST_F(Nav2PluginTest, GazeboTimestampSkewReachesControllerAndGoalChecker)
{
  controller->setPlan(path({{0, 0, 0}, {1, 0, 0}}));
  compute();  // Prime an earlier clock epoch and its cached feedback.
  auto clock = node->get_clock();
  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  geometry_msgs::msg::PoseStamped current;
  current.header.frame_id = "odom";
  current.pose = pose_message({});
  auto request = [&] {
    return controller->computeVelocityCommands(current, geometry_msgs::msg::Twist{}, goal.get());
  };
  auto publish_with_lead = [&](std::int64_t now, std::int64_t lead) {
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), now), RCL_RET_OK);
    current.header.stamp = clock->now();
    observed.header.stamp = rclcpp::Time(now + lead, RCL_ROS_TIME);
    observed.feedback_stamp = observed.header.stamp;
    states->publish(observed);
    spin(20);
  };
  publish_with_lead(10'000'000'000, 15'000'000);
  ASSERT_GT(request().twist.linear.x, 0.);
  publish_with_lead(10'050'000'000, 30'000'000);
  ASSERT_GT(request().twist.linear.x, 0.);

  auto ordered = dynamic_cast<OrderedGoalChecker *>(goal.get());
  ASSERT_NE(ordered, nullptr);
  const geometry_msgs::msg::Twist stopped;
  ordered->setReady(true);
  EXPECT_FALSE(goal->isGoalReached(current.pose, current.pose, stopped));
  publish_with_lead(10'110'000'000, 30'000'000);
  EXPECT_TRUE(goal->isGoalReached(current.pose, current.pose, stopped));

  publish_with_lead(10'650'000'000, 30'000'001);
  EXPECT_THROW(request(), nav2_core::NoValidControl);
  EXPECT_FALSE(goal->isGoalReached(current.pose, current.pose, stopped));
}
TEST_F(Nav2PluginTest, ContinuousVelocityAndOrderedGoalChecker)
{
  controller->setPlan(path({{0, 0, 0}, {0.5, 0.5, 0.5}}));
  const auto speed = compute();
  EXPECT_GT(speed.twist.linear.y, 0);
  EXPECT_NEAR(speed.twist.angular.z, 0, 1e-9);
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
  // Teleporting measured pose does not cancel the last published request.
  // Completion must wait for that request to slew to zero as in a real stop.
  geometry_msgs::msg::TwistStamped stopped_request;
  bool zero_request = false;
  for (int i = 0; i < 20; ++i) {
    stopped_request = compute({0, 0, 0});
    zero_request =
      std::hypot(stopped_request.twist.linear.x, stopped_request.twist.linear.y) < 1e-4 &&
      std::abs(stopped_request.twist.angular.z) < 1e-4;
    EXPECT_FALSE(goal->isGoalReached(origin, origin, geometry_msgs::msg::Twist{}));
    if (zero_request) break;
  }
  ASSERT_TRUE(zero_request);
  std::this_thread::sleep_for(60ms);
  EXPECT_TRUE(goal->isGoalReached(origin, origin, geometry_msgs::msg::Twist{}));
  observed.velocity.linear.z = 1;  // Invalid feedback must clear an earlier stopped sample.
  publish_state();
  EXPECT_FALSE(goal->isGoalReached(origin, origin, geometry_msgs::msg::Twist{}));
}
TEST_F(Nav2PluginTest, GoalCheckerUsesLiveTransformedEndpoint)
{
  publish_state();
  auto ordered = dynamic_cast<OrderedGoalChecker *>(goal.get());
  ASSERT_NE(ordered, nullptr);
  const auto query = pose_message({0, 0, .84});
  const auto historical_target = pose_message({0, 0, .9});
  ordered->setReady(true, Pose{0, 0, .84});
  EXPECT_FALSE(goal->isGoalReached(query, historical_target, geometry_msgs::msg::Twist{}));
  std::this_thread::sleep_for(60ms);
  EXPECT_TRUE(goal->isGoalReached(query, historical_target, geometry_msgs::msg::Twist{}));
  ordered->setReady(true, Pose{0, 0, 1.});
  EXPECT_FALSE(goal->isGoalReached(query, historical_target, geometry_msgs::msg::Twist{}));
  ordered->setReady(true, Pose{0, 0, .84});
  std::this_thread::sleep_for(550ms);
  publish_state();
  EXPECT_FALSE(goal->isGoalReached(query, historical_target, geometry_msgs::msg::Twist{}));
}
TEST_F(Nav2PluginTest, TerminalHoldUsesActiveGoalCheckerTolerances)
{
  constexpr Pose target{1, 0, 0};
  constexpr Pose outside_strict{.965, 0, swerve_chassis_core::pi / 60};
  constexpr Pose inside_strict{.99, 0, .01};
  auto request_at = [&](Pose current) {
    observed.requested_velocity = geometry_msgs::msg::Twist{};
    controller->setPlan(path({current, target}));
    const auto cmd = compute(current).twist;
    return std::hypot(cmd.linear.x, cmd.linear.y) + std::abs(cmd.angular.z);
  };

  EXPECT_LT(request_at(outside_strict), 1e-8);  // Default OrderedGoalChecker.
  node->declare_parameter("strict_goal_checker.xy_goal_tolerance", .02);
  node->declare_parameter("strict_goal_checker.yaw_goal_tolerance", .02);
  goal = goal_loader->createSharedInstance("nav2_controller::SimpleGoalChecker");
  goal->initialize(node, "strict_goal_checker", costmap);
  EXPECT_FALSE(goal->isGoalReached(
    pose_message(outside_strict), pose_message(target), geometry_msgs::msg::Twist{}));
  EXPECT_GT(request_at(outside_strict), 1e-4);
  EXPECT_LT(request_at(inside_strict), 1e-8);

  goal = std::make_shared<GoalCheckerWithoutTolerances>();
  EXPECT_GT(request_at(outside_strict), 1e-4);
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
