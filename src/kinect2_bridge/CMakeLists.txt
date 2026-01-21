cmake_minimum_required(VERSION 3.8)
project(kinect2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DK2_CALIB_PATH="${CMAKE_INSTALL_PREFIX}/../../../src/kinect2_bridge/data/")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kinect2_registration REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(freenect2 REQUIRED)

add_executable(${PROJECT_NAME}_node
    src/kinect2_bridge.cpp
)

target_link_libraries(${PROJECT_NAME}_node freenect2)

include_directories(include/kinect2_bridge)

ament_target_dependencies(${PROJECT_NAME}_node 
                          rclcpp 
                          OpenCV 
                          sensor_msgs 
                          kinect2_registration
                          tf2
                          tf2_ros
                          tf2_geometry_msgs
                          geometry_msgs
                          )

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files so other packages can find them
install(DIRECTORY
  include/kinect2_bridge
  DESTINATION include)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export include directories for other packages
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

