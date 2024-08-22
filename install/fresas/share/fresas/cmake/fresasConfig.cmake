# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fresas_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fresas_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fresas_FOUND FALSE)
  elseif(NOT fresas_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fresas_FOUND FALSE)
  endif()
  return()
endif()
set(_fresas_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fresas_FIND_QUIETLY)
  message(STATUS "Found fresas: 0.0.0 (${fresas_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fresas' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fresas_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fresas_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fresas_DIR}/${_extra}")
endforeach()
