add_executable(tbtrace trace.cpp)

target_link_libraries(tbtrace
  tbutil
  valhalla::valhalla
  )

install(TARGETS tbtrace
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Utilities
  )
