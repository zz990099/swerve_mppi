cmake_minimum_required(VERSION 3.20)
project(swerve_mppi_ros VERSION 0.27.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(swerve_mppi_core 0.26 CONFIG REQUIRED)
foreach(dependency rclcpp rclcpp_lifecycle std_msgs geometry_msgs nav_msgs diagnostic_msgs
    std_srvs builtin_interfaces swerve_gazebo_sim)
  find_package(${dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CircleObstacle.msg" "msg/ObstacleArray.msg" "msg/CommandPublication.msg"
  "srv/ReportApplication.srv"
  DEPENDENCIES std_msgs builtin_interfaces swerve_gazebo_sim)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(swerve_mppi_ros_node SHARED src/conversions.cpp src/observation_buffer.cpp src/controller_node.cpp)
target_compile_features(swerve_mppi_ros_node PUBLIC cxx_std_17)
target_compile_options(swerve_mppi_ros_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(swerve_mppi_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(swerve_mppi_ros_node PUBLIC swerve_mppi::core PRIVATE "${typesupport}")
ament_target_dependencies(swerve_mppi_ros_node PUBLIC rclcpp rclcpp_lifecycle
  nav_msgs diagnostic_msgs std_srvs swerve_gazebo_sim)
add_executable(controller src/main.cpp)
target_link_libraries(controller PRIVATE swerve_mppi_ros_node)

install(TARGETS swerve_mppi_ros_node EXPORT export_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS controller
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_targets(export_node HAS_LIBRARY_TARGET)
ament_export_dependencies(swerve_mppi_core rclcpp rclcpp_lifecycle nav_msgs diagnostic_msgs std_srvs
  swerve_gazebo_sim rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  ament_add_gtest(transport_tests tests/transport_tests.cpp)
  target_link_libraries(transport_tests swerve_mppi_ros_node)
  ament_add_gtest(lifecycle_tests tests/lifecycle_tests.cpp TIMEOUT 60)
  target_link_libraries(lifecycle_tests swerve_mppi_ros_node "${typesupport}")
  ament_target_dependencies(lifecycle_tests rosgraph_msgs)
  set_tests_properties(lifecycle_tests PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=83;RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
ament_package()
