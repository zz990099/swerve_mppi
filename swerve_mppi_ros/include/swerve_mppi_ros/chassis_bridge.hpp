#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace swerve_mppi_ros
{
class ChassisBridge : public rclcpp::Node
{
public:
  explicit ChassisBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ChassisBridge() override;

private:
  struct Runtime;
  std::unique_ptr<Runtime> runtime_;
};
}  // namespace swerve_mppi_ros
