cmake_minimum_required(VERSION 3.22)
project(moveit_srdf)  # Match package name to the folder and package.xml

find_package(ament_cmake REQUIRED)

# Install launch files if the directory exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

