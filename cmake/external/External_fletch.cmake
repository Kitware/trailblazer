#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(AddExternalProject)
define_external_dirs_ex( fletch )
add_external_project_ex( fletch
  GIT_REPOSITORY https://github.com/Kitware/fletch.git
  GIT_TAG master
  #URL "https://github.com/Kitware/fletch/archive/v1.4.0.zip"
  #URL_HASH MD5=dd7f240261c8dc8f1fea4523e700eeac
  CMAKE_CACHE_ARGS
      -Dfletch_BUILD_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -Dfletch_BUILD_CXX11:BOOL=ON
      -Dfletch_ENABLE_Boost:BOOL=ON
      -Dfletch_ENABLE_Eigen:BOOL=ON
      #-Dfletch_ENABLE_GDAL:BOOL=ON
      #-Dfletch_ENABLE_GEOS:BOOL=ON
      #-Dfletch_ENABLE_PNG:BOOL=ON
      -Dfletch_ENABLE_PROJ4:BOOL=ON
      -Dfletch_ENABLE_ZLib:BOOL=ON
      #-Dfletch_ENABLE_libgeotiff:BOOL=ON
      #-Dfletch_ENABLE_libjpeg-turbo:BOOL=ON
      #-Dfletch_ENABLE_libtiff:BOOL=ON
      #-Dfletch_ENABLE_openjpeg:BOOL=ON
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES ""
  #VERBOSE
  )
if (NOT USE_SYSTEM_fletch)
  set(fletch_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/fletch)
  message(STATUS "fletch_DIR : ${fletch_DIR}")
  # Here is where our fletch build libraries are
  set(Eigen3_DIR ${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake)
endif()
