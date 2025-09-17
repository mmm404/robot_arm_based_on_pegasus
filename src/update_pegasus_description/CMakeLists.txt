cmake_minimum_required(VERSION 3.22)
project(update_pegasus_description)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/pegasus_commander.py
  scripts/planned_path_listener.py
  DESTINATION lib/${PROJECT_NAME}
)

