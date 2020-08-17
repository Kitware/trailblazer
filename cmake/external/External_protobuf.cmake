#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------

include(AddExternalProject)
define_external_dirs_ex(protobuf)
add_external_project_ex( protobuf
  URL https://github.com/protocolbuffers/protobuf/releases/download/v3.12.2/protobuf-all-3.12.2.zip
  URL_MD5 ec63c1566640a5873566e76ec8eb1398
  SOURCE_SUBDIR ./cmake
  CMAKE_CACHE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=OFF
    -Dprotobuf_BUILD_TESTS:BOOL=OFF
    -Dprotobuf_BUILD_EXAMPLES:BOOL=OFF
    -Dprotobuf_BUILD_SHARED_LIBS:BOOL=OFF
    -Dprotobuf_MSVC_STATIC_RUNTIME:BOOL=OFF#Don't change MSVC runtime settings (/MD or /MT)
    -Dprotobuf_WITH_ZLIB:BOOL=OFF
  ${PROTOBUF_MULTI_BUILD}
  ${PROTOBUF_MULTI_INSTALL}
  RELATIVE_INCLUDE_PATH "include"
  #DEPENDENCIES ""
  #VERBOSE
)
if (NOT USE_SYSTEM_protobuf)
  if(WIN32)
    set(protobuf_DIR ${CMAKE_INSTALL_PREFIX}/cmake)
  else()
    set(protobuf_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/protobuf)
  endif()
  message(STATUS "protobuf_DIR : ${protobuf_DIR}")
endif()
