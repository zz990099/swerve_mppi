#include "swerve_mppi_ros/controller_node.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swerve_mppi_ros::ControllerNode>());
  rclcpp::shutdown();
}
