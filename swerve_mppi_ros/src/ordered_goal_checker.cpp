#include "swerve_mppi_ros/ordered_goal_checker.hpp"

#include <limits>
#include <pluginlib/class_list_macros.hpp>

#include "swerve_mppi_ros/conversions.hpp"

namespace swerve_mppi_ros
{
void OrderedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  auto node = parent.lock();
  if (!node) throw std::invalid_argument("missing GoalChecker parent");
  clock_ = node->get_clock();
  auto parameter = [&](const std::string & key, double & value) {
    const auto full = name + "." + key;
    if (!node->has_parameter(full)) node->declare_parameter<double>(full, value);
    value = node->get_parameter(full).as_double();
  };
  parameter("xy_goal_tolerance", tolerance_.position);
  parameter("yaw_goal_tolerance", tolerance_.yaw);
  parameter("trans_stopped_velocity", tolerance_.linear_speed);
  parameter("rot_stopped_velocity", tolerance_.angular_speed);
  parameter("settle_time", tolerance_.settle);
  checker_ = swerve_mppi_ros::GoalChecker(tolerance_);
  parameter("state_timeout", state_timeout_);
  if (!std::isfinite(state_timeout_) || state_timeout_ <= 0)
    throw std::invalid_argument("invalid state timeout");
  body_frame_ = costmap->getBaseFrameID();
  const auto topic = name + ".state_topic";
  if (!node->has_parameter(topic))
    node->declare_parameter<std::string>(topic, "chassis_controller/state");
  state_sub_ = node->create_subscription<swerve_chassis_msgs::msg::ChassisState>(
    node->get_parameter(topic).as_string(), rclcpp::SensorDataQoS(),
    [this](const swerve_chassis_msgs::msg::ChassisState & msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (
        msg.header.frame_id != body_frame_ || msg.fault || !msg.feedback_valid ||
        !msg.generator_valid || msg.feedback_stamp.sec < 0 ||
        msg.feedback_stamp.nanosec >= 1000000000) {
        healthy_ = false;
        checker_.reset();
        return;
      }
      const double stamp = seconds(msg.feedback_stamp);
      if (stamp <= feedback_stamp_ && clock_->now().seconds() >= feedback_stamp_) return;
      feedback_stamp_ = stamp;
      feedback_wall_ = std::chrono::steady_clock::now();
      try {
        auto v = velocity(msg.velocity);
        healthy_ = std::hypot(v.vx, v.vy) <= tolerance_.linear_speed &&
                   std::abs(v.wz) <= tolerance_.angular_speed;
        for (double speed : msg.wheel_speeds)
          healthy_ &= std::isfinite(speed) && std::abs(speed) <= 0.05;
      } catch (const std::invalid_argument &) {
      }
      if (!healthy_) checker_.reset();
    });
  reset();
}
void OrderedGoalChecker::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ready_ = false;
  checker_.reset();
}
void OrderedGoalChecker::setReady(bool ready)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ready_ = ready;
  if (!ready) checker_.reset();
}
bool OrderedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query, const geometry_msgs::msg::Pose & target,
  const geometry_msgs::msg::Twist & speed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready_ || !clock_) return false;
  const double age = clock_->now().seconds() - feedback_stamp_;
  if (
    !healthy_ || age < 0 || age > state_timeout_ ||
    std::chrono::duration<double>(std::chrono::steady_clock::now() - feedback_wall_).count() >
      state_timeout_) {
    checker_.reset();
    return false;
  }
  try {
    State state;
    state.pose = pose(query);
    state.velocity = velocity(speed);
    return checker_.reached(state, pose(target), clock_->now().seconds());
  } catch (const std::invalid_argument &) {
    checker_.reset();
    return false;
  }
}
bool OrderedGoalChecker::getTolerances(geometry_msgs::msg::Pose & p, geometry_msgs::msg::Twist & v)
{
  const auto unused = std::numeric_limits<double>::lowest();
  p.position.x = p.position.y = tolerance_.position;
  p.position.z = unused;
  p.orientation = pose_message({0, 0, tolerance_.yaw}).orientation;
  v.linear.x = v.linear.y = tolerance_.linear_speed;
  v.linear.z = unused;
  v.angular.x = v.angular.y = unused;
  v.angular.z = tolerance_.angular_speed;
  return true;
}
}  // namespace swerve_mppi_ros
PLUGINLIB_EXPORT_CLASS(swerve_mppi_ros::OrderedGoalChecker, nav2_core::GoalChecker)
