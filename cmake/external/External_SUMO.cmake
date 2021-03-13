#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(AddExternalProject)
define_external_dirs_ex( SUMO )
add_external_project_ex( SUMO
  GIT_REPOSITORY https://github.com/eclipse/sumo.git
  GIT_TAG v1_8_0
  CMAKE_CACHE_ARGS
      -DEigen3_DIR:PATH=${Eigen3_DIR}
      -DXercesC_DIR:PATH=${XercesC_DIR}
      -DPROJ_INCLUDE_DIR:PATH=${PROJ_INCLUDE_DIR}
      -DPROJ_LIBRARY:PATH=${PROJ_LIBRARY}
      -DENABLE_JAVA_BINDINGS:BOOL=OFF
      -DENABLE_PYTHON_BINDINGS:BOOL=OFF
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES
    fletch
    XercesC
  #VERBOSE
  )
if (NOT USE_SYSTEM_SUMO)
  set(SUMO_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/SUMO)
  message(STATUS "SUMO_DIR : ${SUMO_DIR}")
endif()
