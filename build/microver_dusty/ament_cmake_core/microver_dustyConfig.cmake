# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_microver_dusty_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED microver_dusty_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(microver_dusty_FOUND FALSE)
  elseif(NOT microver_dusty_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(microver_dusty_FOUND FALSE)
  endif()
  return()
endif()
set(_microver_dusty_CONFIG_INCLUDED TRUE)

# output package information
if(NOT microver_dusty_FIND_QUIETLY)
  message(STATUS "Found microver_dusty: 0.0.0 (${microver_dusty_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'microver_dusty' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${microver_dusty_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(microver_dusty_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${microver_dusty_DIR}/${_extra}")
endforeach()
