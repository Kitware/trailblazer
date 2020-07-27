if(WIN32)
  set(expat_depend "expat::libexpat")
else()
  set(expat_depend "expat::expat")
endif()

add_library_ex(readosm
  H_FILES
    readosm.h
    readosm_internals.h
    readosm_protobuf.h
  CPP_FILES
    osm_objects.c
    osmxml.c
    protobuf.c
    readosm.c
  PRIVATE_DEPENDS
    ${expat_depend}
    protobuf::libprotobuf
    ZLIB::ZLIB
)
