project(goal_publisher)

include_directories(
    ${DORA_INCLUDE_DIR}
)

add_executable(goal_publisher_node
    src/goal_publisher_node.cpp
    src/udp_receiver.cpp
)

target_link_libraries(goal_publisher_node
    Threads::Threads
    ${DORA_NODE_API_LIB}
)
