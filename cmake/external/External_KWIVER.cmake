#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(AddExternalProject)
define_external_dirs_ex( KWIVER )
add_external_project_ex( KWIVER
  GIT_REPOSITORY https://github.com/aaron-bray/kwiver.git
  GIT_TAG track_cxx_fix
  #URL "https://github.com/Kitware/kwiver/archive/v1.5.0.zip"
  #URL_HASH MD5=b239799cb98a415a34a90047247226e5
  CMAKE_CACHE_ARGS
      -Dfletch_DIR:PATH=${fletch_DIR}
      -DKWIVER_ENABLE_ARROWS:BOOL=ON
      -DKWIVER_ENABLE_PROJ:BOOL=ON
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES
    fletch
  #VERBOSE
  )
if (NOT USE_SYSTEM_KWIVER)
  set(KWIVER_DIR ${CMAKE_INSTALL_PREFIX}/share/cmake/kwiver)
  message(STATUS "kwiver_DIR : ${KWIVER_DIR}")
endif()
