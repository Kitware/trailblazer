include(ExternalProject)
include(SolveDependencies)

#-----------------------------------------------------------------------------
# Git protocol option
#-----------------------------------------------------------------------------
if(NOT GIT_EXECUTABLE)
  find_package(Git REQUIRED)
endif()

#-----------------------------------------------------------------------------
# CMake global args
#-----------------------------------------------------------------------------
if(MSVC)
  # Allow the MP flag to get set externally
  set(${PROJECT_NAME}_ENABLE_MULTI_PROCESS_BUILDS ON CACHE BOOL "Enable multi-process builds")
  set(PROCESSOR_COUNT "$ENV{NUMBER_OF_PROCESSORS}")
  set(${PROJECT_NAME}_NUM_BUILD_PROCESSES ${PROCESSOR_COUNT} CACHE STRING "The maximum number of processes for the /MP flag")
  if(${PROJECT_NAME}_ENABLE_MULTI_PROCESS_BUILDS)
    set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} /MP${${PROJECT_NAME}_NUM_BUILD_PROCESSES}" CACHE STRING INTERNAL FORCE)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /MP${${PROJECT_NAME}_NUM_BUILD_PROCESSES}" CACHE STRING INTERNAL FORCE)
  endif()
endif()

mark_as_superbuild(
  VARS
    CMAKE_BUILD_TYPE
    CMAKE_DEBUG_POSTFIX
    CMAKE_INSTALL_PREFIX
    #CMAKE_MODULE_PATH # Paths used in include(), find_package()
    #CMAKE_PREFIX_PATH # Path prefixes to be searched, in find_xxx functions
    CMAKE_C_COMPILER
    CMAKE_CXX_COMPILER
    CMAKE_C_FLAGS
    CMAKE_CXX_FLAGS
    CMAKE_EXE_LINKER_FLAGS
    CMAKE_SHARED_LINKER_FLAGS
    CMAKE_CXX_STANDARD
    CMAKE_CXX_STANDARD_REQUIRED
    THREADS_PREFER_PTHREAD_FLAG
    CMAKE_POSITION_INDEPENDENT_CODE
  ALL_PROJECTS
  )

#-----------------------------------------------------------------------------
# CMake args if Apple
#-----------------------------------------------------------------------------
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
  mark_as_superbuild(
    VARS
      CMAKE_OSX_ARCHITECTURES
      CMAKE_OSX_SYSROOT
      CMAKE_OSX_DEPLOYMENT_TARGET
      CMAKE_MACOSX_RPATH
    ALL_PROJECTS
    )
endif()

#-----------------------------------------------------------------------------
# Keep track of include path for superbuild
#-----------------------------------------------------------------------------
set(CMAKE_INCLUDE_PATH )
mark_as_superbuild(VARS CMAKE_INCLUDE_PATH ALL_PROJECTS)

#-----------------------------------------------------------------------------
# Solve dependencies
#-----------------------------------------------------------------------------
set(EXTERNAL_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}) # Location of the "External_*" files

ExternalProject_Include_Dependencies( ${PROJECT_NAME}
  DEPENDS_VAR ${PROJECT_NAME}_DEPENDENCIES
  EP_ARGS_VAR ${PROJECT_NAME}_EP_ARGS
  SUPERBUILD_VAR ${PROJECT_NAME}_SUPERBUILD
  )

if(${PROJECT_NAME}_RUNTIME_PREFIX STREQUAL "")
  set(_runtime_prefix "${CMAKE_INSTALL_PREFIX}")
else()
  set(_runtime_prefix "${${PROJECT_NAME}_RUNTIME_PREFIX}")
endif()

#-----------------------------------------------------------------------------
# Inner build of the main project
#-----------------------------------------------------------------------------
ExternalProject_Add( ${PROJECT_NAME}
  ${${PROJECT_NAME}_EP_ARGS}
  DOWNLOAD_COMMAND ""
  SOURCE_DIR ${${PROJECT_NAME}_SOURCE_DIR}
  BINARY_DIR ${CMAKE_BINARY_DIR}/Innerbuild
  ${COMMON_CMAKE_EP_ARGS}
  CMAKE_ARGS
    -D${PROJECT_NAME}_SUPERBUILD:BOOL=OFF
    -Dexpat_DIR:PATH=${expat_DIR}
    -Dfletch_DIR:PATH=${fletch_DIR}
    -DKWIVER_DIR:PATH=${KWIVER_DIR}
    -Dprotobuf_DIR:PATH=${protobuf_DIR}
    -Dvalhalla_DIR:PATH=${valhalla_DIR}
    -D${PROJECT_NAME}_RUNTIME_PREFIX:PATH=${_runtime_prefix}
  #INSTALL_COMMAND ${SKIP_STEP_COMMAND}
  DEPENDS ${${PROJECT_NAME}_DEPENDENCIES}
)
set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER ${PROJECT_NAME})
