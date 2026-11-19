#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "swerve_mppi_ros/command_stamp_guard.hpp"

namespace swerve_mppi_ros
{
class CmdVelStampGuard : public rclcpp::Node
{
public:
  CmdVelStampGuard() : Node("cmd_vel_stamp_guard")
  {
    output_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    input_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_nav2", 10, [this](geometry_msgs::msg::TwistStamped::UniquePtr command) {
        const auto & stamp = command->header.stamp;
        if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) return;
        const auto source = rclcpp::Time(stamp).nanoseconds();
        const auto & linear = command->twist.linear;
        const auto & angular = command->twist.angular;
        const bool zero = linear.x == 0 && linear.y == 0 && linear.z == 0 && angular.x == 0 &&
                          angular.y == 0 && angular.z == 0;
        const auto outgoing = guard_.accept(source, now().nanoseconds(), zero);
        if (!outgoing) return;
        if (*outgoing != source) command->header.stamp = rclcpp::Time(*outgoing, RCL_ROS_TIME);
        output_->publish(std::move(command));
      });
  }

private:
  CommandStampGuard guard_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr input_;
};
}  // namespace swerve_mppi_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swerve_mppi_ros::CmdVelStampGuard>());
  rclcpp::shutdown();
}
