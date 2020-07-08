include(CommonFindMacros)
include(FindPackageHandleStandardArgs)

setup_find_root_context(ReadOSM)
find_path(ReadOSM_INCLUDE_DIR readosm.h ${ReadOSM_FIND_OPTS})
find_library(ReadOSM_LIBRARY readosm ${ReadOSM_FIND_OPTS})
restore_find_root_context(ReadOSM)

find_package_handle_standard_args(
  ReadOSM REQUIRED_VARS ReadOSM_LIBRARY ReadOSM_INCLUDE_DIR)

if(ReadOSM_FOUND)
  if(NOT TARGET ReadOSM::ReadOSM)
    add_library(ReadOSM::ReadOSM UNKNOWN IMPORTED)
    set_target_properties(ReadOSM::ReadOSM PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${ReadOSM_INCLUDE_DIR}")

    if(EXISTS "${ReadOSM_LIBRARY}")
      set_target_properties(ReadOSM::ReadOSM PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "C"
        IMPORTED_LOCATION "${ReadOSM_LIBRARY}")
    endif()
  endif()
endif()
