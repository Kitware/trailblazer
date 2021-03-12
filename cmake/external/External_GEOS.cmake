#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(AddExternalProject)
define_external_dirs_ex( GEOS )
add_external_project_ex( GEOS
  URL https://github.com/libgeos/geos/archive/3.8.1.zip
  URL_MD5 309cb2ec629dcae7961f22df022c4373
  CMAKE_CACHE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES ""
  #VERBOSE
  )

if (NOT USE_SYSTEM_GEOS)
  set(GEOS_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/GEOS)
  message(STATUS "GEOS_DIR : ${GEOS_DIR}")
endif()
