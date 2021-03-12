#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------

include(AddExternalProject)
define_external_dirs_ex( LuaJIT )
add_external_project_ex( LuaJIT
  GIT_REPOSITORY  https://github.com/WohlSoft/LuaJIT.git
  GIT_TAG v2.1
  CMAKE_CACHE_ARGS
    # None
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES ""
  #VERBOSE
  )

