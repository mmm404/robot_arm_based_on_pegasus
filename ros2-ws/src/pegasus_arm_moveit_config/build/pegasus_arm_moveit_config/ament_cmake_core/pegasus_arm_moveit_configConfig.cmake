# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pegasus_arm_moveit_config_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pegasus_arm_moveit_config_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pegasus_arm_moveit_config_FOUND FALSE)
  elseif(NOT pegasus_arm_moveit_config_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pegasus_arm_moveit_config_FOUND FALSE)
  endif()
  return()
endif()
set(_pegasus_arm_moveit_config_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pegasus_arm_moveit_config_FIND_QUIETLY)
  message(STATUS "Found pegasus_arm_moveit_config: 0.3.0 (${pegasus_arm_moveit_config_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pegasus_arm_moveit_config' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pegasus_arm_moveit_config_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pegasus_arm_moveit_config_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pegasus_arm_moveit_config_DIR}/${_extra}")
endforeach()
