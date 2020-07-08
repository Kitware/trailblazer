cmake_minimum_required(VERSION 3.15)
project(trailblazer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

list(PREPEND CMAKE_MODULE_PATH ${trailblazer_SOURCE_DIR}/cmake)

find_package(Boost 1.51 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kwiver REQUIRED)
find_package(ReadOSM REQUIRED)
find_package(valhalla REQUIRED)

include(GenerateExportHeader)

# Work around broken dependencies
set_property(TARGET kwiver::vital APPEND PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")

# Main library
add_library(tbutil
  graph.cpp
  reader.cpp
  util.cpp
  )

generate_export_header(tbutil)
target_include_directories(tbutil
  PUBLIC
  $<BUILD_INTERFACE:${trailblazer_BINARY_DIR}>
  )

target_link_libraries(tbutil
  PUBLIC
  kwiver::vital
  PRIVATE
  kwiver::vital_vpm
  ReadOSM::ReadOSM
  )

# Command-line utilities
add_executable(tbtrace trace.cpp)
target_include_directories(tbtrace PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(tbtrace
  tbutil
  valhalla::valhalla
  )
