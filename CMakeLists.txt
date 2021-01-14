sc_begin_package(DETECT)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/libs)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../base/common/libs)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../base/common/libs)

subdirs(src)
