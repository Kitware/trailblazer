cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_EXTENSIONS False)
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /EHsc")
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(UNIX AND NOT APPLE)
  set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/")
endif()

if(NOT CMAKE_INSTALL_PREFIX OR CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  message(STATUS "Setting install dir inside build dir ${CMAKE_BINARY_DIR} ")
  set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "Install location" FORCE)
else()
  message(STATUS "Using preset install directory ${CMAKE_INSTALL_PREFIX}")
endif()

#------------------------------------------------------------------------------
# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  mark_as_advanced(CMAKE_BUILD_TYPE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_DEBUG_POSTFIX "d")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

project(Trailblazer VERSION 3.0.0 LANGUAGES C CXX)

#-----------------------------------------------------------------------------
# Project install directories
#-----------------------------------------------------------------------------
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "Install location" FORCE)
endif()
set(CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX})
# Let's go ahead and make these directories
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/bin)
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/include)
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)

#-----------------------------------------------------------------------------
# Update CMake module path & cmake dir
#-----------------------------------------------------------------------------
set(CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/utilities
    ${CMAKE_INSTALL_PREFIX}
    ${CMAKE_INSTALL_PREFIX}/lib/cmake # Many packages install their cmake to lib
    )
set(${PROJECT_NAME}_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#-----------------------------------------------------------------------------
# SUPERBUILD OPTIONS
#-----------------------------------------------------------------------------
option(${PROJECT_NAME}_SUPERBUILD "Initial pull and build of all dependent libraries/executables" ON)

if(${PROJECT_NAME}_SUPERBUILD)

  #-----------------------------------------------------------------------------
  # Define External dependencies
  #-----------------------------------------------------------------------------
  macro(define_dependency extProj)
    list(APPEND ${PROJECT_NAME}_DEPENDENCIES ${extProj})
    option(USE_SYSTEM_${extProj} "Exclude ${extProj} from superbuild and use an existing build." OFF)
    mark_as_advanced(USE_SYSTEM_${extProj})
  endmacro()
  
  #define_dependency(CURL)
  define_dependency(expat)
  define_dependency(XercesC)
  define_dependency(fletch)
  define_dependency(kwiver)
  define_dependency(protobuf)
  define_dependency(valhalla)
  define_dependency(SUMO)
  
  if(${PROJECT_NAME}_DEPENDENCIES)
    #-----------------------------------------------------------------------------
    # Solve project dependencies
    #-----------------------------------------------------------------------------
    # Call CMakeLists.txt in cmake/external which will solve the dependencies
    # and add the external projects, including this one: this top-level
    # CMakeLists.txt will be called back with SUPERBUILD=OFF, to execute
    # the rest of the code below (INNERBUILD), which explains the `return`
    add_subdirectory(cmake/external)
    return()
  else()
    message(STATUS "No dependencies, skipping superbuild")
  endif()
endif()

#-----------------------------------------------------------------------------
#                               INNERBUILD
#-----------------------------------------------------------------------------

if(MSVC)  
# Using MD as that seems to be what I run into alot, you could change these to /MT and /MTd if you want...
  set(CMAKE_CXX_FLAGS_DEBUG "/D_DEBUG /MDd /Zi /Ob2 /Oi /Od /RTC1 /MP /EHsc" CACHE STRING INTERNAL FORCE)
  set(CMAKE_CXX_FLAGS_RELEASE "/MD /MP /EHsc" CACHE STRING INTERNAL FORCE)
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MDd /Zi /MP /EHsc" CACHE STRING INTERNAL FORCE)
endif()
if ( CMAKE_COMPILER_IS_GNUCC )
  # Protobuf is not using the same variable name in its swap specificiation definitions
  # Resulting in a LOT of warnings per file that includes protobuf headers, slowing the build down
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-attributes")
  if(CMAKE_BUILD_TYPE EQUAL "Debug")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj")
  endif()
endif()


set(${PROJECT_NAME}_INSTALL_FOLDER ${PROJECT_NAME}-${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR})

include(Find)# Our find macros
include(AddLibrary)# Extras to do when creating a library
include(AddExecutable)# Extras to do when creating an executable

#-----------------------------------------------------------------------------
# Find external dependencies
#-----------------------------------------------------------------------------
find_package(valhalla REQUIRED)
find_package(kwiver REQUIRED)
find_package(Boost 1.51 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(expat REQUIRED NO_DEFAULT_PATH)

# Work around broken dependencies
set_property(TARGET kwiver::vital APPEND PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")

add_subdirectory(src)

#--------------------------------------------------------------------------
# Add setup script for *nix systems
#--------------------------------------------------------------------------
if(NOT WIN32)
  # Create setup shell script to create an environment for running examples
  set(LIBRARY_PATH_VAR "LD_LIBRARY_PATH")
  if( APPLE )
    set(LIBRARY_PATH_VAR "DYLD_FALLBACK_LIBRARY_PATH")
  endif()
endif()

#-----------------------------------------------------------------------------
# This variable controls the prefix used to generate the following files:
#  ${PROJECT_NAME}ConfigVersion.cmake
#  ${PROJECT_NAME}Config.cmake
#  ${PROJECT_NAME}Targets.cmake
# and it also used to initialize ${PROJECT_NAME}_INSTALL_CONFIG_DIR value.
set(export_config_name ${PROJECT_NAME})
set(${PROJECT_NAME}_INSTALL_CONFIG_DIR "lib/cmake/${${PROJECT_NAME}_INSTALL_FOLDER}")
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${${PROJECT_NAME}_INSTALL_CONFIG_DIR})
#------------------------------------------------------------------------------
# Configure ${PROJECT_NAME}ConfigVersion.cmake common to build and install tree
include(CMakePackageConfigHelpers)
set(config_version_file ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake)
write_basic_package_version_file(
  ${config_version_file}
  VERSION "${${PROJECT_NAME}_VERSION}"
  COMPATIBILITY ExactVersion
  )
#------------------------------------------------------------------------------
# Export '${PROJECT_NAME}Targets.cmake' for a build tree
export(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE Pulse::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
  )

# Configure '${PROJECT_NAME}Config.cmake' for a build tree
set(build_config ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake)
configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in 
  ${build_config}
  INSTALL_DESTINATION "${PROJECT_BINARY_DIR}"
  )

#------------------------------------------------------------------------------
# Export '${PROJECT_NAME}Targets.cmake' for an install tree
install(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE Pulse::
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${${PROJECT_NAME}_INSTALL_CONFIG_DIR}
  )

set(install_config ${PROJECT_BINARY_DIR}/CMakeFiles/${PROJECT_NAME}Config.cmake)
configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in 
  ${install_config}
  INSTALL_DESTINATION ${${PROJECT_NAME}_INSTALL_CONFIG_DIR}
  )

# Install config files
install(
  FILES ${config_version_file} ${install_config}
  DESTINATION "${${PROJECT_NAME}_INSTALL_CONFIG_DIR}"
  )
