cmake_minimum_required(VERSION 3.15)
project(trailblazer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

list(PREPEND CMAKE_MODULE_PATH ${trailblazer_SOURCE_DIR}/cmake)

find_package(Boost 1.51 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kwiver REQUIRED)
find_package(ReadOSM REQUIRED)
find_package(valhalla REQUIRED)

# Work around broken dependencies
set_property(TARGET kwiver::vital APPEND PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")

add_executable(tbconvert convert.cpp)
target_link_libraries(tbconvert
  kwiver::vital_vpm
  kwiver::vital
  ReadOSM::ReadOSM)

add_executable(tbtrace trace.cpp)
target_include_directories(tbtrace PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(tbtrace valhalla::valhalla)
