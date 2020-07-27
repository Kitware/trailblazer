
add_library_ex(tbutil
  H_FILES
    csv.h
    graph.h
    reader.h
    types.h
    util.h
  CPP_FILES
    csv.cpp
    graph.cpp
    reader.cpp
    util.cpp
  PUBLIC_DEPENDS
    kwiver::vital
    readosm
  PRIVATE_DEPENDS
    kwiver::vital_vpm
    protobuf::libprotobuf
)

# Command-line utilities
add_executable_ex(tbtrace trace.cpp)
target_include_directories(tbtrace PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(tbtrace
  tbutil
  valhalla::valhalla
)
