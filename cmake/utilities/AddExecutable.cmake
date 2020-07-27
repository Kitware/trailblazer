
function(add_executable_ex target)
  set (files ${ARGN})
  list(LENGTH files num_files)
    if (${num_files} EQUAL 0)
        message ("No files associated with target ${target}")
    endif ()
  add_executable(${target} ${files})
  set_target_properties(${target} PROPERTIES
    DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
  set_target_properties (${target} PROPERTIES 
    FOLDER ${PROJECT_NAME})
  add_custom_command(TARGET ${target} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${target}> ${CMAKE_INSTALL_PREFIX}/bin)
  foreach(f ${files})
    list(APPEND target_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
  endforeach()
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${target_FILES})

  if(MSVC) # Configure running executable out of MSVC
    set_property(TARGET ${target} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_INSTALL_PREFIX}/bin")
  endif()
  
  

endfunction()
