cmake_minimum_required(VERSION 3.8)
project(pegasus_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY urdf URDF srdf config rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()