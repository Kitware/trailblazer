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
    ReadOSM::readosm
    kwiver::vital_vpm
)

tb_install_headers(
  graph.h
  reader.h
  types.h
  util.h
  )

tb_install_library(tbutil)
