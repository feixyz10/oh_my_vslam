add_subdirectory(core)
add_subdirectory(frontend)
add_subdirectory(backend)
add_subdirectory(visualizer)

file(GLOB SRC_FILES "*.cc" "*.h")
add_library(slam SHARED ${SRC_FILES})

add_executable(slam_test slam_test.cpp)
target_link_libraries(slam_test slam frontend backend core common ${THIRD_PARTY_LIBS})