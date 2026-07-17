project(dora_mickrobot)

include_directories(
    include
    ${DORA_INCLUDE_DIR}
)

add_executable(mickrobotx4 src/mickx4_bringup.cc)

target_link_libraries(mickrobotx4
    yaml-cpp
    ${DORA_NODE_API_LIB}
    serial
    Threads::Threads
)
