file(GLOB SRC_FILES "*.cc" "*.h")

add_library(backend SHARED ${SRC_FILES})

add_executable(backend_test backend_test.cpp)
target_link_libraries(backend_test backend core common ${THIRD_PARTY_LIBS})
