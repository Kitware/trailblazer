#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(AddExternalProject)
define_external_dirs_ex( XercesC )
add_external_project_ex( XercesC
  URL "https://github.com/apache/xerces-c/archive/v3.2.3.zip"
  URL_HASH MD5=7e2e0baace31b8e369cf2fb040306262
  CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES ""
  #VERBOSE
  )
if (NOT USE_SYSTEM_XercesC)
  set(XercesC_DIR ${CMAKE_INSTALL_PREFIX}/cmake)
  message(STATUS "XercesC_DIR : ${XercesC_DIR}")
endif()