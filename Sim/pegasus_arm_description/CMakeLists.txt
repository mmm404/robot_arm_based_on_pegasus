cmake_minimum_required(VERSION 3.8)
project(pegasus_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# Dependencies
###############################################################################
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)

# If you use Python-based nodes or scripts that import rclpy, etc.:
# find_package(rclpy REQUIRED)
# find_package(moveit_ros_planning REQUIRED)
# find_package(moveit_ros_move_group REQUIRED)

###############################################################################
# Testing
###############################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
# Install Directories
###############################################################################
# 1) Install URDF folder
install(
  DIRECTORY URDF
  DESTINATION share/${PROJECT_NAME}
)

# 2) Install launch folder
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 3) Install rviz folder
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

###############################################################################
# Install Python Scripts
###############################################################################
# If you want to install your Python scripts so they can be run as executables,
# do the following:
install(
  PROGRAMS
    src/pegasus_arm_commander/pegasus_arm_commander/pegasus_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

###############################################################################
# ament_package
###############################################################################
ament_package()

