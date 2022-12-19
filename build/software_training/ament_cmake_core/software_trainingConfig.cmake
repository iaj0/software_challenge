# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_software_training_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED software_training_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(software_training_FOUND FALSE)
  elseif(NOT software_training_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(software_training_FOUND FALSE)
  endif()
  return()
endif()
set(_software_training_CONFIG_INCLUDED TRUE)

# output package information
if(NOT software_training_FIND_QUIETLY)
  message(STATUS "Found software_training: 0.0.0 (${software_training_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'software_training' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${software_training_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(software_training_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${software_training_DIR}/${_extra}")
endforeach()
