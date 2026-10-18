#include "swerve_mppi_ros/chassis_bridge.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swerve_mppi_ros::ChassisBridge>());
  rclcpp::shutdown();
  return 0;
}
