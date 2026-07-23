project(astar_planner_dora)

include_directories(
    include
    ${DORA_INCLUDE_DIR}
)

add_executable(astar_planner_dora_node
    src/astar_planner_dora_node.cpp
    src/astar_algorithm.cpp
    src/map_loader.cpp
    src/coordinate_transform.cpp
    src/json_serialization.cpp
    src/path_smoother.cpp
)

target_link_libraries(astar_planner_dora_node
    Threads::Threads
    nlohmann_json::nlohmann_json
    ${DORA_NODE_API_LIB}
)
