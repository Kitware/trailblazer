cmake_minimum_required(VERSION 3.12)

project(Trailblazer VERSION 0.0.3 LANGUAGES CXX)

# Subdirectory into which headers should be installed
set(Trailblazer_INCLUDEDIR trailblazer)

#------------------------------------------------------------------------------
# Superbuild
#------------------------------------------------------------------------------
option(${PROJECT_NAME}_SUPERBUILD
  "Build ourself and our dependencies as subprojects"
  ON
  )

if(${PROJECT_NAME}_SUPERBUILD)
  include(cmake/superbuild.cmake)

  if(${PROJECT_NAME}_DEPENDENCIES)
    # Transfer to cmake/external/CMakeLists.txt which will drive building our
    # dependencies, and ourself, as external projects, and then halt further
    # evaluation; the external project for ourself will reinvoke this same
    # CMakeLists.txt with SUPERBUILD=OFF
    add_subdirectory(cmake/external)
    return()
  else()
    message(STATUS "No dependencies are enabled, skipping superbuild")
  endif()
endif()

#------------------------------------------------------------------------------
# Regular build
#------------------------------------------------------------------------------

# Update CMake module path
list(PREPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/utilities
  )

# Include various utility modules
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)
include(GNUInstallDirs)
include(TargetUtilities)

# Include project configuration and dependencies
include(cmake/config.cmake)
include(cmake/dependencies.cmake)

add_subdirectory(src)

#------------------------------------------------------------------------------
# Common and build tree exports
#------------------------------------------------------------------------------

# Generate CMake package configuration file
configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  @ONLY
  )
write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion
  )

# Export targets for build tree
export(
  EXPORT ${PROJECT_NAME}_EXPORTS
  NAMESPACE ${PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
  )

#------------------------------------------------------------------------------
# Install tree exports
#------------------------------------------------------------------------------

set(${PROJECT_NAME}_INSTALL_FOLDER
  ${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
  )

# This variable controls the prefix used to generate the following files:
#  ${PROJECT_NAME}ConfigVersion.cmake
#  ${PROJECT_NAME}Config.cmake
#  ${PROJECT_NAME}Targets.cmake
set(${PROJECT_NAME}_INSTALL_CONFIG_DIR
  "${CMAKE_INSTALL_LIBDIR}/cmake/${${PROJECT_NAME}_INSTALL_FOLDER}")

install(
  EXPORT ${PROJECT_NAME}_EXPORTS
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${${PROJECT_NAME}_INSTALL_CONFIG_DIR}
  COMPONENT Development
  )

install(
  FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION "${${PROJECT_NAME}_INSTALL_CONFIG_DIR}"
  COMPONENT Development
  )
