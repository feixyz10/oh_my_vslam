file(GLOB SRC_FILES "*.cc" "*.h")

add_library(frontend SHARED ${SRC_FILES})

add_executable(feature_tracker_test feature_tracker_test.cpp)
target_link_libraries(feature_tracker_test frontend core common ${THIRD_PARTY_LIBS})

add_executable(vo_test vo_test.cpp)
target_link_libraries(vo_test frontend core common ${THIRD_PARTY_LIBS})

add_executable(frontend_test frontend_test.cpp)
target_link_libraries(frontend_test frontend core common ${THIRD_PARTY_LIBS})