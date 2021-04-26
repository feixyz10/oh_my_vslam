file(GLOB SRC_FILES "*.cc" "*.h")

add_library(core SHARED ${SRC_FILES})

add_executable(camera_test camera_test.cpp)
target_link_libraries(camera_test ${THIRD_PARTY_LIBS} core)

