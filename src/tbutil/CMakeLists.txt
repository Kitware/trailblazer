tb_add_library(tbutil
  csv.cpp
  graph.cpp
  reader.cpp
  util.cpp
  )

target_link_libraries(tbutil
  PUBLIC
    kwiver::vital
  PRIVATE
    readosm
    kwiver::vital_vpm
)

tb_install_headers(
  graph.h
  reader.h
  types.h
  util.h
  )

tb_install_library(tbutil)

# Command-line utilities
add_executable_ex(tbtrace trace.cpp)
target_include_directories(tbtrace PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(tbtrace
  tbutil
  valhalla::valhalla
)
