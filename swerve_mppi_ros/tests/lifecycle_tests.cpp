#include <gtest/gtest.h>

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "swerve_mppi_ros/controller_node.hpp"
#include "swerve_mppi_ros/conversions.hpp"
#include "swerve_mppi_ros/msg/command_publication.hpp"
#include "swerve_mppi_ros/msg/obstacle_array.hpp"
#include "swerve_mppi_ros/srv/report_application.hpp"

namespace
{
using namespace swerve_mppi_ros;
using namespace std::chrono_literals;

class Lifecycle : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    chassis = std::make_shared<rclcpp::Node>("swerve_controller");
    for (const auto & parameter : std::vector<std::pair<std::string, double>>{
           {"wheelbase", 0.6},
           {"track_width", 0.5},
           {"wheel_radius", 0.1},
           {"update_rate", 100},
           {"cmd_timeout", 0.5},
           {"max_wheel_speed", 20},
           {"max_wheel_acceleration", 40},
           {"max_steering_rate", 2.5},
           {"steering_limit", 1.5707963267948966},
           {"max_linear_speed", 0.8},
           {"max_angular_speed", 0.8},
           {"max_linear_acceleration", 0.9},
           {"max_angular_acceleration", 1.3},
           {"drive_steering_limit", 0.2},
           {"steering_alignment_tolerance", 0.05},
           {"steering_alignment_duration", 0.05},
           {"mode_switch_timeout", 5},
           {"stopped_wheel_speed", 0.05}}) {
      chassis->declare_parameter(parameter.first, parameter.second);
    }
    for (const auto & parameter : std::vector<std::pair<std::string, std::string>>{
           {"frame_prefix", ""},
           {"odom_frame", ""},
           {"odom_child_frame", ""},
           {"odom_topic", "odom"}}) {
      chassis->declare_parameter(parameter.first, parameter.second);
    }
    chassis->declare_parameter("publish_odom", true);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    controller = std::make_shared<ControllerNode>(options);
    clock_pub = chassis->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    state_pub = chassis->create_publisher<swerve_gazebo_sim::msg::ChassisState>(
      "chassis_controller/state", rclcpp::SensorDataQoS());
    odom_pub = chassis->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    path_pub = chassis->create_publisher<nav_msgs::msg::Path>("path", 1);
    obstacles_pub = chassis->create_publisher<msg::ObstacleArray>("obstacles", 1);
    command_sub = chassis->create_subscription<swerve_gazebo_sim::msg::ChassisCommand>(
      "chassis_controller/command", 10,
      [this](const swerve_gazebo_sim::msg::ChassisCommand & command) {
        commands.push_back(command);
      });
    publication_sub = chassis->create_subscription<msg::CommandPublication>(
      "/mppi_controller/command_publication", 10,
      [this](const msg::CommandPublication & publication) { publications.push_back(publication); });
    diagnostic_sub = chassis->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/mppi_controller/diagnostics", 10,
      [this](const diagnostic_msgs::msg::DiagnosticArray & diagnostic) {
        if (!diagnostic.status.empty()) last_status = diagnostic.status.front();
      });
    evidence =
      chassis->create_client<srv::ReportApplication>("/mppi_controller/report_application");
    recovery = chassis->create_client<std_srvs::srv::Trigger>("/mppi_controller/recover_chassis");
    executor.add_node(chassis);
    executor.add_node(controller->get_node_base_interface());
    spin(100ms);
  }
  void TearDown() override
  {
    executor.remove_node(controller->get_node_base_interface());
    executor.remove_node(chassis);
    controller.reset();
    chassis.reset();
  }
  void spin(std::chrono::milliseconds duration = 25ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }
  void clock()
  {
    rosgraph_msgs::msg::Clock message;
    message.clock = ros_time(stamp);
    clock_pub->publish(message);
    spin(5ms);
  }
  void sample(bool task = false, bool obstacle = true)
  {
    stamp += 20000000;
    clock();
    swerve_gazebo_sim::msg::ChassisState state;
    state.header.stamp = ros_time(stamp);
    state.header.frame_id = "base_footprint";
    state.request_id = request_id;
    state.phase = faulted ? state.FAULT : state.READY;
    state.fault = faulted ? state.COMMAND_TIMEOUT : state.NO_FAULT;
    state.confirmed = !faulted;
    state_pub->publish(state);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ros_time(stamp);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.orientation.w = 1;
    odom_pub->publish(odom);
    if (obstacle) {
      msg::ObstacleArray message;
      message.header = odom.header;
      obstacles_pub->publish(message);
    }
    if (task) {
      nav_msgs::msg::Path path;
      path.header = odom.header;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.orientation.w = 1;
      path.poses.push_back(pose);
      path_pub->publish(path);
    }
    spin();
  }
  void configure()
  {
    EXPECT_EQ(controller->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    for (int i = 0; i < 8; ++i) sample(i == 0);
  }
  srv::ReportApplication::Response::SharedPtr report(bool correct = true, int variant = 0)
  {
    auto request = std::make_shared<srv::ReportApplication::Request>();
    request->command_id = publications.back().command_id + (correct ? 0 : 1);
    request->published_stamp = publications.back().published_stamp;
    request->earliest_application = publications.back().published_stamp;
    request->latest_application = publications.back().published_stamp;
    if (variant == 1)
      request->published_stamp = ros_time(source_time(request->published_stamp) - 1);
    if (variant == 2)
      request->earliest_application = ros_time(source_time(request->earliest_application) - 1);
    if (variant == 3) request->latest_application = ros_time(stamp + 1000000000);
    if (variant == 4)
      request->latest_application = ros_time(source_time(request->earliest_application) - 1);
    if (variant == 5)
      request->latest_application = ros_time(source_time(request->earliest_application) + 30000000);
    auto future = evidence->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> chassis;
  std::shared_ptr<ControllerNode> controller;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  rclcpp::Publisher<swerve_gazebo_sim::msg::ChassisState>::SharedPtr state_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<msg::ObstacleArray>::SharedPtr obstacles_pub;
  rclcpp::Subscription<swerve_gazebo_sim::msg::ChassisCommand>::SharedPtr command_sub;
  rclcpp::Subscription<msg::CommandPublication>::SharedPtr publication_sub;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_sub;
  rclcpp::Client<srv::ReportApplication>::SharedPtr evidence;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recovery;
  std::vector<swerve_gazebo_sim::msg::ChassisCommand> commands;
  std::vector<msg::CommandPublication> publications;
  diagnostic_msgs::msg::DiagnosticStatus last_status;
  int64_t stamp = 1000000000;
  uint64_t request_id = 0;
  bool faulted = false;
};

TEST_F(Lifecycle, ActivationRequiresClockVerifiedParametersAndExplicitContext)
{
  EXPECT_EQ(controller->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  for (int i = 0; i < 5; ++i) sample(false, false);
  EXPECT_EQ(controller->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  sample(true);
  EXPECT_TRUE(commands.empty());
  EXPECT_EQ(controller->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  sample();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front().velocity.linear.x, 0);
  EXPECT_EQ(commands.front().header.frame_id, "base_footprint");
}

TEST_F(Lifecycle, RejectsSharedParameterMismatch)
{
  chassis->set_parameter(rclcpp::Parameter("wheel_radius", 0.2));
  configure();
  EXPECT_EQ(controller->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(commands.empty());
}

TEST_F(Lifecycle, ApplicationIdentityAndWindowGateNextPublication)
{
  configure();
  ASSERT_EQ(controller->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  sample();
  ASSERT_EQ(publications.size(), 1u);
  EXPECT_FALSE(report(false)->accepted);
  for (int i = 0; i < 5; ++i) sample();
  EXPECT_EQ(commands.size(), 1u);
  EXPECT_FALSE(report(true, 1)->accepted);
  EXPECT_FALSE(report(true, 2)->accepted);
  EXPECT_FALSE(report(true, 3)->accepted);
  EXPECT_FALSE(report(true, 4)->accepted);
  EXPECT_FALSE(report(true, 5)->accepted);
  EXPECT_TRUE(report()->accepted);
  EXPECT_FALSE(report()->accepted);
  sample();
  EXPECT_EQ(commands.size(), 2u);
  EXPECT_EQ(publications.front().command, commands.front());
  EXPECT_GT(
    source_time(publications.back().published_stamp),
    source_time(publications.front().published_stamp));
  EXPECT_EQ(controller->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  for (int i = 0; i < 5; ++i) sample();
  EXPECT_EQ(commands.size(), 2u);
  EXPECT_EQ(controller->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(Lifecycle, RejectsInvalidTaskAndClockDomainChanges)
{
  configure();
  EXPECT_FALSE(controller->set_parameter(rclcpp::Parameter("use_sim_time", false)).successful);
  controller->activate();
  sample();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_TRUE(report()->accepted);
  nav_msgs::msg::Path invalid;
  invalid.header.frame_id = "map";
  path_pub->publish(invalid);
  for (int i = 0; i < 6; ++i) sample();
  EXPECT_EQ(commands.size(), 1u);
  EXPECT_EQ(last_status.level, last_status.ERROR);
}

TEST_F(Lifecycle, MissingApplicationEvidenceLatchesUntilReconfiguration)
{
  configure();
  controller->activate();
  sample();
  ASSERT_EQ(commands.size(), 1u);
  for (int i = 0; i < 14; ++i) sample();
  EXPECT_EQ(commands.size(), 1u);
  EXPECT_EQ(last_status.level, last_status.ERROR);
  EXPECT_FALSE(report()->accepted);
}

TEST_F(Lifecycle, ClockPauseAndRollbackStopPublication)
{
  configure();
  controller->activate();
  sample();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_TRUE(report()->accepted);
  spin(300ms);
  EXPECT_EQ(last_status.level, last_status.ERROR);
  sample();
  EXPECT_EQ(commands.size(), 1u);
  controller->deactivate();
  controller->cleanup();
  configure();
  controller->activate();
  sample();
  ASSERT_EQ(commands.size(), 2u);
  stamp -= 1000000000;
  clock();
  spin();
  EXPECT_EQ(last_status.level, last_status.ERROR);
  EXPECT_EQ(commands.size(), 2u);
}

TEST_F(Lifecycle, SourceAndObstacleExpiryStopPublication)
{
  configure();
  controller->activate();
  sample();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_TRUE(report()->accepted);
  for (int i = 0; i < 10; ++i) {
    stamp += 20000000;
    clock();
    spin();
  }
  EXPECT_EQ(last_status.level, last_status.ERROR);
  EXPECT_EQ(commands.size(), 1u);
  controller->deactivate();
  controller->cleanup();
  configure();
  controller->activate();
  sample();
  for (int i = 0; i < 20; ++i) {
    if (!publications.empty()) report();
    sample(false, false);
  }
  EXPECT_EQ(last_status.level, last_status.ERROR);
}

TEST_F(Lifecycle, ExplicitChassisRecoveryRetriesOneZeroRequest)
{
  configure();
  controller->activate();
  sample();
  request_id = 9;
  faulted = true;
  sample();
  auto future = recovery->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);
  for (int i = 0; i < 12; ++i) sample();
  ASSERT_GE(commands.size(), 3u);
  for (size_t i = 1; i < commands.size(); ++i) {
    EXPECT_EQ(commands[i].request_id, 10u);
    EXPECT_EQ(commands[i].velocity, geometry_msgs::msg::Twist());
    EXPECT_EQ(commands[i].entry_velocity, commands[1].entry_velocity);
  }
  request_id = 10;
  faulted = false;
  sample();
  const auto count = commands.size();
  for (int i = 0; i < 6; ++i) sample();
  EXPECT_EQ(commands.size(), count);
  EXPECT_EQ(publications.size(), 1u);
}
}  // namespace
