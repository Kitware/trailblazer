include(FindPackageHandleStandardArgs)
#-----------------------------------------------------------------------------
# Look for header directory
#-----------------------------------------------------------------------------

macro(find_header_ex package header)

  # Subdir to the include directory is optional
  set(sub_dir "")
  set (extra_macro_args ${ARGN})
  # Did we get any optional args?
  list(LENGTH extra_macro_args num_extra_args)
  if (${num_extra_args} GREATER 0)
    list(GET extra_macro_args 0 sub_dir)
  endif ()
  
  # Should I look in system locations?
  set(use_system)
  foreach(arg IN LISTS extra_macro_args)
    if(arg STREQUAL "USE_SYSTEM")
      set(use_system TRUE)
    endif()
  endforeach()

  if(NOT use_system)
    unset(_SEARCH_DIR)
    if(${package}_ROOT_DIR)
      set(_SEARCH_DIR ${${package}_ROOT_DIR}/${sub_dir})
    else()
      set(_SEARCH_DIR ${CMAKE_INSTALL_PREFIX}/include/${sub_dir})
    endif()
    #message(STATUS "Searching for : ${_SEARCH_DIR}/${header}")

    find_path(${package}_INCLUDE_DIR
      NAMES
        ${header}
      PATHS
        ${_SEARCH_DIR}
      NO_DEFAULT_PATH
    )
  else()
    #message(STATUS "I am looking in system for ${package}")
    find_path(${package}_INCLUDE_DIR
      NAMES
        ${header}
    )
  endif()
  #message(STATUS "Found it : ${${package}_INCLUDE_DIR}/${header}")
  

  if (EXISTS ${${package}_INCLUDE_DIR}/${header})
    string(TOUPPER ${package} PACKAGE)
    list(APPEND ${PACKAGE}_INCLUDE_DIRS ${${package}_INCLUDE_DIR})
    mark_as_advanced(${PACKAGE}_INCLUDE_DIRS)
  else()
    message(FATAL_ERROR "Could not find ${${package}_INCLUDE_DIR}/${header}")
  endif()
  #message(STATUS "${package}_INCLUDE_DIRS : ${${package}_INCLUDE_DIRS}")
  #unset(${package}_INCLUDE_DIR CACHE)
endmacro()

#-----------------------------------------------------------------------------
# Look for both optimized and debug external libraries
#-----------------------------------------------------------------------------

macro(find_libary_ex package library)

  # Debug postfix argument is optional
  set(release_postfix)
  set(debug_postfix ${CMAKE_DEBUG_POSTFIX})
  set (extra_macro_args ${ARGN})
  # Did we get any optional args?
  list(LENGTH extra_macro_args num_extra_args)
  if (${num_extra_args} EQUAL 1)
    list(GET extra_macro_args 0 debug_postfix)
    string(STRIP ${debug_postfix} debug_postfix)
    #message(STATUS "${package} changing debug_postfix to ${debug_postfix}")
  elseif(${num_extra_args} EQUAL 2)
    list(GET extra_macro_args 0 release_postfix)
    string(STRIP ${release_postfix} release_postfix)
    list(GET extra_macro_args 1 debug_postfix)
    string(STRIP ${debug_postfix} debug_postfix)
    message(STATUS "Looking for ${package} libraries ${library}${release_postfix} and ${library}${debug_postfix}")
  endif()
  
  # Should I look in system locations?
  set(use_system)
  foreach(arg IN LISTS extra_macro_args)
    if(arg STREQUAL "USE_SYSTEM")
      set(use_system TRUE)
    endif()
  endforeach()
  
  if(NOT use_system)
    unset(_SEARCH_DIR)
    if(${package}_ROOT_DIR)
      if(${package}_LIB_DIR)
        set(_SEARCH_DIR ${${package}_ROOT_DIR}/${${package}_LIB_DIR})
      endif()
      #message(STATUS "Looking for ${package} libs in ${_SEARCH_DIR}")
    else()
      set(_SEARCH_DIR ${CMAKE_INSTALL_PREFIX}/lib)
    endif()
    
    
    unset(${PACKAGE}_LIBRARY_${library}-RELEASE CACHE)
    string(TOUPPER ${package} PACKAGE)
    find_library(${PACKAGE}_LIBRARY_${library}-RELEASE
      NAMES
        ${library}${release_postfix}
        lib${library}${release_postfix}
      PATHS
        ${_SEARCH_DIR}
        ${_SEARCH_DIR}/Release
      NO_DEFAULT_PATH
    )
    
    unset(${PACKAGE}_LIBRARY_${library}-DEBUG CACHE)
    find_library(${PACKAGE}_LIBRARY_${library}-DEBUG
      NAMES
        ${library}${debug_postfix}
        lib${library}${debug_postfix}
      PATHS
        ${_SEARCH_DIR}
        ${_SEARCH_DIR}/Debug
      NO_DEFAULT_PATH
    )
    
  else()
    unset(${PACKAGE}_LIBRARY_${library}-RELEASE CACHE)
    find_library(${PACKAGE}_LIBRARY_${library}-RELEASE
      NAMES
        ${library}${release_postfix}
        lib${library}${release_postfix}
    )
    
    unset(${PACKAGE}_LIBRARY_${library}-DEBUG CACHE)
    find_library(${PACKAGE}_LIBRARY_${library}-DEBUG
      NAMES
        ${library}${debug_postfix}
        lib${library}${debug_postfix}
    )

  endif()

  if (EXISTS ${${PACKAGE}_LIBRARY_${library}-RELEASE})
    #message(STATUS "${PACKAGE}_LIBRARY_${library}-RELEASE : ${${PACKAGE}_LIBRARY_${library}-RELEASE}")
    list(APPEND ${PACKAGE}_LIBRARIES optimized ${${PACKAGE}_LIBRARY_${library}-RELEASE})
    list(APPEND ${PACKAGE}_RELEASE_LIBRARIES ${${PACKAGE}_LIBRARY_${library}-RELEASE})
  endif()
  mark_as_advanced(${PACKAGE}_LIBRARY_${library}-RELEASE)
  #message(STATUS "Libraries : ${${PACKAGE}_RELEASE_LIBRARIES}")

  if (EXISTS ${${PACKAGE}_LIBRARY_${library}-DEBUG})
    #message(STATUS "${PACKAGE}_LIBRARY_${library}-DEBUG : ${${PACKAGE}_LIBRARY_${library}-DEBUG}")
    list(APPEND ${PACKAGE}_LIBRARIES debug ${${PACKAGE}_LIBRARY_${library}-DEBUG})
    list(APPEND ${PACKAGE}_DEBUG_LIBRARIES ${${PACKAGE}_LIBRARY_${library}-DEBUG})
  endif()
  mark_as_advanced(${PACKAGE}_LIBRARY_${library}-DEBUG)
  #message(STATUS "Libraries : ${${PACKAGE}_DEBUG_LIBRARIES}")
endmacro()

#-----------------------------------------------------------------------------
# Confirm the package was found and was complete
#-----------------------------------------------------------------------------
macro(find_package_ex package)

  string(TOUPPER ${package} PACKAGE)
  if (${PACKAGE}_FOUND)
    return()
  endif()

  find_package_handle_standard_args(${PACKAGE}
    REQUIRED_VARS
      ${PACKAGE}_INCLUDE_DIRS
      ${PACKAGE}_LIBRARIES)

  # custom target package name is optional
  set(tgt_name ${package})
  set (extra_macro_args ${ARGN})
  # Did we get any optional args?
  list(LENGTH extra_macro_args num_extra_args)
  if (${num_extra_args} GREATER 0)
    list(GET extra_macro_args 0 tgt_name)
  endif ()

  #if(${PACKAGE}_FOUND AND NOT TARGET ${tgt_name})
    add_library(${tgt_name} INTERFACE IMPORTED)
    target_include_directories(${tgt_name} INTERFACE "${${PACKAGE}_INCLUDE_DIRS}")
    foreach(lib ${${PACKAGE}_DEBUG_LIBRARIES})
      target_link_libraries(${tgt_name} INTERFACE debug ${lib})
    endforeach()
    foreach(lib ${${PACKAGE}_RELEASE_LIBRARIES})
      target_link_libraries(${tgt_name} INTERFACE optimized ${lib})
    endforeach()
  #endif()
  #message(STATUS "Target ${tgt_name} libs : ${${PACKAGE}_LIBRARIES}")
endmacro()

#-----------------------------------------------------------------------------
# Confirm the (header only) package was found and was complete
#-----------------------------------------------------------------------------
macro(find_header_package_ex package)

  string(TOUPPER ${package} PACKAGE)
  if (${PACKAGE}_FOUND)
    return()
  endif()

  find_package_handle_standard_args(${PACKAGE}
    REQUIRED_VARS
      ${PACKAGE}_INCLUDE_DIRS)

  # custom target package name is optional
  set(tgt_name ${package})
  set (extra_macro_args ${ARGN})
  # Did we get any optional args?
  list(LENGTH extra_macro_args num_extra_args)
  if (${num_extra_args} GREATER 0)
    list(GET extra_macro_args 0 tgt_name)
  endif ()

  if(${PACKAGE}_FOUND AND NOT TARGET ${tgt_name})
    add_library(${tgt_name} INTERFACE IMPORTED)
    target_include_directories(${tgt_name} INTERFACE "${${PACKAGE}_INCLUDE_DIRS}")
  endif()
endmacro()