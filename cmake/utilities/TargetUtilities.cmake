include(CMakeParseArguments)

# -----------------------------------------------------------------------------
function(tb_source_group)
  set(_sources "")
  foreach(file IN LISTS ARGN)
    get_filename_component(path ${file} ABSOLUTE)
    list(APPEND _sources "${path}")
  endforeach()
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${_sources})
endfunction()

# -----------------------------------------------------------------------------
function(tb_copy_runtime TARGET)
  if(NOT ${PROJECT_NAME}_RUNTIME_PREFIX STREQUAL "")
    get_target_property(_type ${TARGET} TYPE)
    if(_type STREQUAL "STATIC_LIBRARY")
      add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
          $<TARGET_FILE:${TARGET}>
          ${${PROJECT_NAME}_RUNTIME_PREFIX}/${CMAKE_INSTALL_LIBDIR})
    elseif(_type STREQUAL "SHARED_LIBRARY" OR _type STREQUAL "MODULE_LIBRARY")
      add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
          $<TARGET_FILE:${TARGET}>
          ${${PROJECT_NAME}_RUNTIME_PREFIX}/${CMAKE_INSTALL_BINDIR})
      if(WIN32 AND _type STREQUAL "SHARED_LIBRARY")
        add_custom_command(
          TARGET ${TARGET} POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_LINKER_FILE:${TARGET}>
            ${${PROJECT_NAME}_RUNTIME_PREFIX}/${CMAKE_INSTALL_LIBDIR})
      endif()
    elseif(_type STREQUAL "EXECUTABLE")
      add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
          $<TARGET_FILE:${TARGET}>
          ${${PROJECT_NAME}_RUNTIME_PREFIX}/${CMAKE_INSTALL_BINDIR})
    endif()
  endif()
endfunction()

# -----------------------------------------------------------------------------
function(tb_add_library NAME)
  set(_multival_arguments
    SOURCES
    PRIVATE_HEADERS
    PUBLIC_HEADERS
    PRIVATE_LINK_LIBRARIES
    PUBLIC_LINK_LIBRARIES
    )
  cmake_parse_arguments(
    "" # prefix (empty)
    "" # options (none)
    "" # single-value arguments (none)
    "${_multival_arguments}"
    ${ARGN}
    )

  # Build library
  add_library(${NAME} ${_SOURCES} ${_PUBLIC_HEADERS} ${_PRIVATE_HEADERS})
  generate_export_header(${NAME})
  tb_copy_runtime(${NAME})

  # Set interface include directories
  target_include_directories(${NAME}
    PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
      $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
      $<INSTALL_INTERFACE:${install_includedir}>
    )

  # Link to dependencies
  target_link_libraries(${NAME}
    PRIVATE ${_PRIVATE_LINK_LIBRARIES}
    PUBLIC ${_PUBLIC_LINK_LIBRARIES}
    )

  # Install headers
  foreach(header ${_PUBLIC_HEADERS})
    get_filename_component(dest ${header} DIRECTORY)
    if(DEST STREQUAL "")
      set(dest ${${PROJECT_NAME}_INCLUDEDIR})
    else()
      set(dest ${${PROJECT_NAME}_INCLUDEDIR}/${dest})
    endif()
    install(FILES ${header}
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${dest}
      COMPONENT Development
      )
  endforeach()

  # Install library
  install(TARGETS ${NAME} EXPORT ${PROJECT_NAME}_EXPORTS
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT RuntimeLibraries
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT RuntimeLibraries
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development
    )

  # Add sources to IDE source group
  set_target_properties(${NAME} PROPERTIES FOLDER ${PROJECT_NAME})
  tb_source_group(${_SOURCES} ${_PUBLIC_HEADERS} ${_PRIVATE_HEADERS})
endfunction()

# -----------------------------------------------------------------------------
function(tb_add_executable NAME)
  add_executable(${NAME} ${ARGN})
  tb_copy_runtime(${NAME})

  target_include_directories(${NAME}
    PRIVATE ${${PROJECT_NAME}_INCLUDE_FOLDERS}
    )

  set_target_properties(${NAME} PROPERTIES
    FOLDER ${PROJECT_NAME}
    )

  tb_source_group(${ARGN})

  # Configure running executable out of MSVC
  if(MSVC AND NOT ${PROJECT_NAME}_RUNTIME_PREFIX STREQUAL "")
    set_property(TARGET ${NAME}
      PROPERTY VS_DEBUGGER_WORKING_DIRECTORY
      "${${PROJECT_NAME}_RUNTIME_PREFIX}/${CMAKE_INSTALL_BINDIR}"
      )
  endif()
endfunction()
