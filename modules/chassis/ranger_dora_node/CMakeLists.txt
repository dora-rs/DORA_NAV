project(ranger_dora_node)

# ── Include directories ──
include_directories(
    include
    ugv_sdk/include
    ${DORA_INCLUDE_DIR}
)

# ── Build ugv_sdk as static library ──
add_subdirectory(ugv_sdk)

add_executable(ranger_miniv3_node
    src/ranger_bringup.cpp
)

target_link_libraries(ranger_miniv3_node
    ugv_sdk
    ${DORA_NODE_API_LIB}
    Threads::Threads
)
