sc_begin_package(DETECT)

SET(VERSION 1.0.0)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/libs)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../base/common/libs)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../base/common/libs)

option(
  "SCDETECT_DEBUG" "Build scdetect with additional debug output enabled"
  OFF)
if (${SCDETECT_DEBUG})
  add_definitions(-DSCDETECT_DEBUG)
endif()


subdirs(src doc)
