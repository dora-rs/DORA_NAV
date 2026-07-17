project(pure_pursuit_dora)

include_directories(
    include
    ${DORA_INCLUDE_DIR}
)

add_executable(pure_pursuit_dora_node
    src/pure_pursuit_dora_node.cpp
    src/pure_pursuit_component.cpp
)

target_link_libraries(pure_pursuit_dora_node
    nlohmann_json::nlohmann_json
    ${DORA_NODE_API_LIB}
    Threads::Threads
)
