#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(AddExternalProject)
define_external_dirs_ex( ReadOSM )
add_external_project_ex( ReadOSM
  GIT_REPOSITORY https://github.com/aaron-bray/readosm.git
  GIT_TAG cmake
  CMAKE_CACHE_ARGS
      -Dexpat_DIR:PATH=${expat_DIR}
      -Dprotobuf_DIR:PATH=${protobuf_DIR}
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES
    fletch
    expat
    protobuf
  #VERBOSE
  )
if (NOT USE_SYSTEM_ReadOSM)
  set(ReadOSM_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/ReadOSM)
  message(STATUS "ReadOSM_DIR : ${ReadOSM_DIR}")
endif()
