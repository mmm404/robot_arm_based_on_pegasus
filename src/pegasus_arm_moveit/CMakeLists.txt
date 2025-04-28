cmake_minimum_required(VERSION 3.22)
project(pegasus_arm_moveit)

find_package(ament_cmake REQUIRED)

# Install the config folder, which contains the SRDF
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# (Optional) Install the launch folder if you have one
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
