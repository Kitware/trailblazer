#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
set(_make_command make)
if(MSVC)
  set(_make_command nmake)
elseif(MINGW)
  set(_make_command mingw32-make)
endif()
include(AddExternalProject)
define_external_dirs_ex( SpatiaLite )
add_external_project_ex( SpatiaLite
  URL http://www.gaia-gis.it/gaia-sins/libspatialite-5.0.0.zip
  URL_MD5 4e6c5f60aa19b4bb6988879cf6cbf427
  PATCH_COMMAND
    COMMAND ${CMAKE_COMMAND} -DQwt_INSTALL_DIR=${Qwt_PREFIX}/install
                             -DQwt_SOURCE_DIR=${Qwt_SOURCE_DIR}
                             -DQwt_PATCH_DIR=${CMAKE_SOURCE_DIR}/cmake/patches/Qwt
      -P ${CMAKE_SOURCE_DIR}/cmake/patches/Qwt/patch.cmake
  CONFIGURE_COMMAND
    Qt5::qmake ${Qwt_SOURCE_DIR}/qwt.pro
  BUILD_COMMAND
    ${_make_command}
  INSTALL_COMMAND
    ${_make_command} install QWT_INSTALL_PREFIX=${Qwt_BINARY_DIR}
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES ""
  #VERBOSE
  )

if (NOT USE_SYSTEM_Qwt)
  set(Qwt_ROOT_DIR ${Qwt_PREFIX}/install)
  message(STATUS "Qwt_ROOT_DIR : ${Qwt_ROOT_DIR}")
endif()
