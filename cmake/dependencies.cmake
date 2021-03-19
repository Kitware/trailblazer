# ReadOSM
if(DEFINED ReadOSM_DIR)
  find_package(ReadOSM CONFIG REQUIRED)
else()
  find_package(ReadOSM CONFIG QUIET)
  if(NOT ReadOSM_FOUND)
    find_package(ReadOSM REQUIRED)
  endif()
endif()

# KWIVER (requires Eigen3)
find_package(Eigen3 REQUIRED)
find_package(kwiver REQUIRED)

# Work around broken dependencies
set_property(TARGET kwiver::vital APPEND PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")

# Valhalla (requires Boost to use)
find_package(Boost 1.51 REQUIRED)
find_package(valhalla REQUIRED)
find_package(SUMO REQUIRED)
