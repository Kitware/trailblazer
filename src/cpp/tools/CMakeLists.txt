tb_add_executable(tbtrace
  SOURCES
    trace.cpp
  LINK_LIBRARIES
    tbutil
  )

tb_add_executable(tbroute
  SOURCES
    route.cpp
  LINK_LIBRARIES
    tbutil
  )
  
tb_add_executable(tbsim
  SOURCES
    simulate.cpp
  LINK_LIBRARIES
    SUMO::libsumocpp
    kwiver::vital
  )
