include(CMakeParseArguments)

# -----------------------------------------------------------------------------
function(tb_source_group NAME)
  set(_sources "")
  foreach(file IN LISTS ARGN)
    get_filename_component(path ${file} ABSOLUTE)
    list(APPEND _sources "${path}")
  endforeach()
  source_group(${NAME} FILES ${_sources})
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
  add_library(${NAME} ${_SOURCES})
  generate_export_header(${NAME})

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
  tb_source_group(${NAME} ${_SOURCES} ${_PUBLIC_HEADERS} ${_PRIVATE_HEADERS})
endfunction()

# -----------------------------------------------------------------------------
function(tb_add_executable NAME)
  add_executable(${NAME} ${ARGN})

  target_include_directories(${NAME}
    PRIVATE ${${PROJECT_NAME}_INCLUDE_FOLDERS}
    )

  set_target_properties(${NAME} PROPERTIES
    FOLDER ${PROJECT_NAME}
    )

  tb_source_group(${NAME} ${ARGN})

  # Configure running executable out of MSVC
  if(MSVC)
    set_property(TARGET ${NAME} PROPERTY
      VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_INSTALL_PREFIX}/bin"
      )
  endif()
endfunction()
