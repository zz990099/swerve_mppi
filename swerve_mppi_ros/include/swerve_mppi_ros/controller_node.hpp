#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
namespace swerve_mppi_ros
{
class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControllerNode() override;

private:
  struct Runtime;
  std::unique_ptr<Runtime> runtime_;
};
}  // namespace swerve_mppi_ros
