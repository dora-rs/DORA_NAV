project(dwa_planner_dora)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${DORA_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(dwa_planner_dora_node
    src/dwa_planner_dora_node.cpp
    src/pointcloud_processor.cpp
    src/costmap_builder.cpp
    src/dwa_planner_component.cpp
)

target_link_libraries(dwa_planner_dora_node
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${DORA_NODE_API_LIB}
    Threads::Threads
)
