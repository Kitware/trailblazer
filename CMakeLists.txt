cmake_minimum_required(VERSION 3.11)
project(trailblazer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(Boost 1.51 REQUIRED)

find_package(valhalla REQUIRED)

add_executable(tbtrace trace.cpp)
target_include_directories(tbtrace PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(tbtrace valhalla::valhalla)
