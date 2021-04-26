file(GLOB SRC_FILES "*.cc" "*.h")

add_library(frontend SHARED ${SRC_FILES})

add_executable(feature_extractor_test feature_extractor_test.cpp)
target_link_libraries(feature_extractor_test core common ${THIRD_PARTY_LIBS})