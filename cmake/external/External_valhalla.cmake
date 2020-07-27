#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(AddExternalProject)
define_external_dirs_ex( valhalla )
add_external_project_ex( valhalla
  #URL "https://github.com/valhalla/valhalla/archive/3.0.9.zip"
  #URL_HASH MD5=6b5c03c7825fefdc63004f1cf8cd1eea
  #GIT_REPOSITORY https://github.com/mwoehlke-kitware/valhalla.git
  GIT_REPOSITORY https://github.com/aaron-bray/valhalla.git
  GIT_TAG export-targets
  CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -Dprotobuf_DIR:PATH=${protobuf_DIR}
      -DCURL_DIR:PATH=${CURL_DIR}
      -DENABLE_DATA_TOOLS:BOOL=OFF
      -DENABLE_HTTP:BOOL=OFF
      -DENABLE_NODE:BOOL=OFF
      -DENABLE_NODE_BINDINGS:BOOL=OFF
      -DENABLE_PYTHON_BINDINGS:BOOL=OFF
      -DENABLE_SERVICES:BOOL=OFF
      -DENABLE_TESTS:BOOL=OFF
      -DENABLE_TOOLS:BOOL=OFF
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES
    CURL
    protobuf
    fletch
  #VERBOSE
  )
if (NOT USE_SYSTEM_valhalla)
  set(valhalla_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/valhalla)
  message(STATUS "valhalla_DIR : ${valhalla_DIR}")
endif()