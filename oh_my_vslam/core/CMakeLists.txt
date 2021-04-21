aux_source_directory(. SRC_FILES)

add_library(core SHARED ${SRC_FILES})

add_executable(camera_test
    camera_test.cc
)
target_link_libraries(camera_test
    gtest_main
    core
    ${OpenCV_LIBS}
)