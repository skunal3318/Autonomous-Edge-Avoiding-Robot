# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_edge_ws_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED edge_ws_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(edge_ws_FOUND FALSE)
  elseif(NOT edge_ws_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(edge_ws_FOUND FALSE)
  endif()
  return()
endif()
set(_edge_ws_CONFIG_INCLUDED TRUE)

# output package information
if(NOT edge_ws_FIND_QUIETLY)
  message(STATUS "Found edge_ws: 0.0.0 (${edge_ws_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'edge_ws' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT edge_ws_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(edge_ws_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${edge_ws_DIR}/${_extra}")
endforeach()
