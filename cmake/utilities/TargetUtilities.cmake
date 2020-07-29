include(CMakeParseArguments)

#------------------------------------------------------------------------------
function(tb_add_library NAME)
  add_library(${NAME} ${ARGN})

  set(install_includedir
    ${CMAKE_INSTALL_INCLUDEDIR}/${${PROJECT_NAME}_INCLUDEDIR}
    )

  generate_export_header(${NAME})
  target_include_directories(${NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
      $<INSTALL_INTERFACE:${install_includedir}>
    )
endfunction()

#------------------------------------------------------------------------------
function(tb_install_headers)
  foreach(header ${ARGN})
    get_filename_component(dest ${header} DIRECTORY)
    if(DEST STREQUAL "")
      set(dest ${${PROJECT_NAME}_INCLUDEDIR})
    else()
      set(dest ${${PROJECT_NAME}_INCLUDEDIR}/${dest})
    endif()
    install(FILES ${ARGN}
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${dest}
      COMPONENT Development
      )
  endforeach()
endfunction()

function(tb_install_library NAME)
  install(TARGETS ${NAME} EXPORT ${PROJECT_NAME}_EXPORTS
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT RuntimeLibraries
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT RuntimeLibraries
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development
    )
endfunction()
