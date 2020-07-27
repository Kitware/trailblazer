
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
    expat::libexpat
    protobuf::libprotobuf
    ZLIB::ZLIB
)