#pragma once

#include <memory>
#include <nav2_core/controller.hpp>

namespace swerve_mppi_ros
{
class Nav2Controller : public nav2_core::Controller
{
public:
  Nav2Controller();
  ~Nav2Controller() override;
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  bool cancel() override;
  void reset() override;

private:
  struct Runtime;
  std::unique_ptr<Runtime> runtime_;
};
}  // namespace swerve_mppi_ros
