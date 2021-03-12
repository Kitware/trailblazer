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
define_external_dirs_ex( SQLite3 )
add_external_project_ex( SQLite3
  URL https://www.sqlite.org/2020/sqlite-amalgamation-3330000.zip
  URL_MD5 61a8cae35ab6201d916304ec4a6f06b8
  PATCH_COMMAND
    COMMAND ${CMAKE_COMMAND} -DQwt_INSTALL_DIR=${Qwt_PREFIX}/install
                             -DQwt_SOURCE_DIR=${Qwt_SOURCE_DIR}
                             -DQwt_PATCH_DIR=${CMAKE_SOURCE_DIR}/cmake/patches/Qwt
      -P ${CMAKE_SOURCE_DIR}/cmake/patches/Qwt/patch.cmake
  #INSTALL_COMMAND
  #  ${_make_command} install QWT_INSTALL_PREFIX=${Qwt_BINARY_DIR}
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES ""
  #VERBOSE
  )

if (NOT USE_SYSTEM_SQLite3)
  set(Qwt_ROOT_DIR ${Qwt_PREFIX}/install)
  message(STATUS "Qwt_ROOT_DIR : ${Qwt_ROOT_DIR}")
endif()
