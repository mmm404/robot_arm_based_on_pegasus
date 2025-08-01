# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_admittance_controller_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED admittance_controller_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(admittance_controller_FOUND FALSE)
  elseif(NOT admittance_controller_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(admittance_controller_FOUND FALSE)
  endif()
  return()
endif()
set(_admittance_controller_CONFIG_INCLUDED TRUE)

# output package information
if(NOT admittance_controller_FIND_QUIETLY)
  message(STATUS "Found admittance_controller: 4.25.0 (${admittance_controller_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'admittance_controller' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT admittance_controller_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(admittance_controller_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${admittance_controller_DIR}/${_extra}")
endforeach()
