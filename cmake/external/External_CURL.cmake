#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(AddExternalProject)
define_external_dirs_ex( CURL )
add_external_project_ex( CURL
  URL "https://github.com/curl/curl/archive/curl-7_71_1.zip"
  URL_HASH MD5=d54ae0b8661f820b16e7240dfe71a124
  CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES ""
  #VERBOSE
  )
if (NOT USE_SYSTEM_CURL)
  set(CURL_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/CURL)
  message(STATUS "CURL_DIR : ${CURL_DIR}")
endif()