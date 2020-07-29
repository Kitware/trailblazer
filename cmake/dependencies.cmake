# ReadOSM (built internally for now; requires expat, protobuf, zlib)
find_package(expat REQUIRED NO_DEFAULT_PATH)
find_package(ZLIB REQUIRED)

# KWIVER (requires Eigen3)
find_package(Eigen3 REQUIRED)
find_package(kwiver REQUIRED)

# Work around broken dependencies
set_property(TARGET kwiver::vital APPEND PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")

# Valhalla (requires Boost to use)
find_package(Boost 1.51 REQUIRED)
find_package(valhalla REQUIRED)

