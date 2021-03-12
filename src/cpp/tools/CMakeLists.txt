tb_add_executable(tbtrace trace.cpp)
# TODO Support keywords like add_library_ex
target_link_libraries(tbtrace
  tbutil
  )

tb_add_executable(tbroute route.cpp)

target_link_libraries(tbroute
  tbutil
  )

install(TARGETS tbtrace tbroute
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Utilities
  )
