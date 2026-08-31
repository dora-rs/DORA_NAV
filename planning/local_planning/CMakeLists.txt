cmake_minimum_required(VERSION 3.10)
project(local_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    $ENV{HOME}/dora/apis/c/node/
    $ENV{HOME}/dora/apis/c/operator/
    /usr/local/include
)

add_executable(local_planner_node src/local_planner.cpp)
target_link_libraries(local_planner_node 
    $ENV{HOME}/dora/target/release/libdora_node_api_c.a
)
