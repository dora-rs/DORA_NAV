project(ndt_mapping_dora)

add_subdirectory(3rdparty/ndt_cpu)

include_directories(
    include
    3rdparty/ndt_cpu/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${DORA_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ndt_mapping_dora_node
    src/ndt_mapping_dora_node.cpp
    src/ndt_mapper.cpp
)

target_link_libraries(ndt_mapping_dora_node
    ${PCL_LIBRARIES}
    ndt_cpu
    yaml-cpp
    ${DORA_NODE_API_LIB}
    Threads::Threads
)