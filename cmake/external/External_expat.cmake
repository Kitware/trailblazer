#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(AddExternalProject)
define_external_dirs_ex( expat )
add_external_project_ex( expat
  URL "https://github.com/libexpat/libexpat/archive/R_2_2_9.zip"
  URL_HASH MD5=b28ad5d282bbcacda16bb9d30145ceed
  SOURCE_SUBDIR ./expat
  CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES ""
  #VERBOSE
  )
if (NOT USE_SYSTEM_expat)
  set(expat_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/expat-2.2.9)
  message(STATUS "expat_DIR : ${expat_DIR}")
endif()