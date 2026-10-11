#pragma once

#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace swerve_mppi_ros
{
// Run with serialized callbacks; the provided executable uses a single executor.
class ControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControllerNode() override;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  struct Runtime;
  std::shared_ptr<Runtime> runtime_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};
}  // namespace swerve_mppi_ros
