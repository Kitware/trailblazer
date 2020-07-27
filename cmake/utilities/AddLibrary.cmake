macro(_subdir_list result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

include(GenerateExportHeader)
function(add_library_ex target)

  set(options VERBOSE SHARED LIB_INSTALL_ONLY)
  set(oneValueArgs)
  set(multiValueArgs H_FILES CPP_FILES SUBDIR_LIST PUBLIC_DEPENDS PRIVATE_DEPENDS INSTALL_HEADER_DIR)
  include(CMakeParseArguments)
  cmake_parse_arguments(target "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  message(STATUS "Configuring ${target}")

  #-----------------------------------------------------------------------------
  # Verbose (display arguments)
  #-----------------------------------------------------------------------------
  if(target_VERBOSE)
    foreach(opt ${options} ${oneValueArgs} ${multiValueArgs})
      message(STATUS "${opt}:${target_${opt}}")
    endforeach()
  endif()

  #-----------------------------------------------------------------------------
  # Get files and directories
  #-----------------------------------------------------------------------------
  if( NOT target_H_FILES AND NOT target_CPP_FILES )
    message(STATUS "Auto adding source files")
    file(GLOB_RECURSE target_H_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    file(GLOB_RECURSE target_CPP_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
    file(GLOB_RECURSE testing_FILES "${CMAKE_CURRENT_SOURCE_DIR}/test/*")
    if(testing_FILES)
      list(REMOVE_ITEM target_H_FILES ${testing_FILES})
      list(REMOVE_ITEM target_CPP_FILES ${testing_FILES})
    endif()
  endif()
  
  if( NOT target_SUBDIR_LIST )
    _subdir_list(target_SUBDIR_LIST ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  list(APPEND target_BUILD_INTERFACE_LIST "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")
  list(APPEND target_BUILD_INTERFACE_LIST "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  foreach(subdir ${target_SUBDIR_LIST})
    if( NOT ${subdir} STREQUAL "test")
      list(APPEND target_BUILD_INTERFACE_LIST "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${subdir}>")
    endif()
  endforeach()

  #-----------------------------------------------------------------------------
  # Create target (library)
  #-----------------------------------------------------------------------------
  set(target_LIB_TYPE STATIC)
  if(target_SHARED)
    set(target_LIB_TYPE SHARED)
  endif()
  
  add_library( ${target} ${target_LIB_TYPE}
    ${target_H_FILES}
    ${target_CPP_FILES}
    )
    

  if(target_SHARED)
    add_custom_command(TARGET ${target} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${target}> ${CMAKE_INSTALL_PREFIX}/bin)
  else()
    add_custom_command(TARGET ${target} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${target}> ${CMAKE_INSTALL_PREFIX}/lib)
  endif()

  set_target_properties(${target} PROPERTIES PREFIX "")

  #-----------------------------------------------------------------------------
  # Link libraries to current target
  #-----------------------------------------------------------------------------
  # Add dependent public targets
  foreach(d ${target_PUBLIC_DEPENDS})
    list(APPEND ${target}_PUBLIC_LIBRARIES "${d}")
  endforeach()
  if(target_VERBOSE)
    message(STATUS "${target} using public libraries : ${${target}_PUBLIC_LIBRARIES}")
  endif()
  target_link_libraries( ${target} PUBLIC
    ${${target}_PUBLIC_LIBRARIES}
    )
  # Add dependent private targets
  foreach(d ${target_PRIVATE_DEPENDS})
    list(APPEND ${target}_PRIVATE_LIBRARIES "${d}")
  endforeach()
  if(target_VERBOSE)
    message(STATUS "${target} using private libraries : ${${target}_PRIVATE_LIBRARIES}")
  endif()
  target_link_libraries( ${target} PRIVATE
    ${${target}_PRIVATE_LIBRARIES}
    )

  #-----------------------------------------------------------------------------
  # Include directories
  #-----------------------------------------------------------------------------
  generate_export_header(${target})
  target_include_directories( ${target} PUBLIC
    ${target_BUILD_INTERFACE_LIST}
    $<INSTALL_INTERFACE:include/${${PROJECT_NAME}_INSTALL_FOLDER}>
    )

  #-----------------------------------------------------------------------------
  # Set compile flags for the target
  #-----------------------------------------------------------------------------
  target_compile_options(${target} PRIVATE
                           $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
                                -Wall>
                           $<$<CXX_COMPILER_ID:MSVC>:
                                -W4 -MP>)

  if(NOT target_LIB_INSTALL_ONLY)
    #-----------------------------------------------------------------------------
    # Install headers
    #-----------------------------------------------------------------------------
    foreach(h ${target_H_FILES})
      #message(STATUS "Header at ${h}")
      get_filename_component(DEST_DIR ${h} DIRECTORY)
      #message(STATUS "Going to ${target_INSTALL_HEADER_DIR}/${DEST_DIR}")
      install(FILES
        ${h}
        DESTINATION include/${${PROJECT_NAME}_INSTALL_FOLDER}/${target_INSTALL_HEADER_DIR}/${DEST_DIR}
        COMPONENT Development
      )
    endforeach()
  endif()
  
  #-----------------------------------------------------------------------------
  # Install library
  #-----------------------------------------------------------------------------
  install( TARGETS ${target} EXPORT ${PROJECT_NAME}Targets
    RUNTIME DESTINATION bin COMPONENT RuntimeLibraries
    LIBRARY DESTINATION lib COMPONENT RuntimeLibraries
    ARCHIVE DESTINATION lib COMPONENT Development
    )

  #-----------------------------------------------------------------------------
  # Add the target to project folders
  #-----------------------------------------------------------------------------
  set_target_properties (${target} PROPERTIES FOLDER ${PROJECT_NAME})
  foreach(h ${target_H_FILES})
    list(APPEND target_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${h}")
  endforeach()
  foreach(cpp ${target_CPP_FILES})
    list(APPEND target_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${cpp}")
  endforeach()
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${target_FILES})

endfunction()
