tb_add_executable(tbtrace trace.cpp)
# TODO Support keywords like add_library_ex
target_link_libraries(tbtrace
  tbutil
  )
