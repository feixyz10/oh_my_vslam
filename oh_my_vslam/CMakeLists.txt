add_subdirectory(core)
add_subdirectory(frontend)
add_subdirectory(backend)

aux_source_directory(. SRC_FILES)

# add_library(oh_my_vslam SHARED ${SRC_FILES})
