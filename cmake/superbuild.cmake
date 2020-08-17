enable_language(C)
enable_language(CXX)

include(${CMAKE_CURRENT_LIST_DIR}/config.cmake)

list(PREPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_LIST_DIR}/external
  ${CMAKE_CURRENT_LIST_DIR}/utilities
  )

function(define_dependency NAME)
  string(TOUPPER "${NAME}" NAME_UC)
  option(USE_SYSTEM_${NAME_UC}
    "Exclude ${NAME} from superbuild and use an external version."
    OFF
    )
  if(NOT USE_SYSTEM_${NAME_UC})
    list(APPEND ${PROJECT_NAME}_DEPENDENCIES ${NAME})
    set(${PROJECT_NAME}_DEPENDENCIES
      "${${PROJECT_NAME}_DEPENDENCIES}" PARENT_SCOPE
      )
  endif()
endfunction()

define_dependency(expat)
define_dependency(fletch)
define_dependency(KWIVER)
define_dependency(protobuf)
define_dependency(ReadOSM)
define_dependency(SUMO)
define_dependency(valhalla)
define_dependency(XercesC)
