cmake_minimum_required(VERSION 3.10)
project(pointcloud_merger)

#========================
#  dora
#========================
if(NOT DORA_INCLUDE_DIR)
  set(DORA_INCLUDE_DIR $ENV{HOME}/dora/apis/c/node; $ENV{HOME}/dora/apis/c/operator)
endif()
if(NOT DORA_NODE_API_LIB)
  set(DORA_NODE_API_LIB $ENV{HOME}/dora/target/release/libdora_node_api_c.a)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${DORA_INCLUDE_DIR}
)

add_executable(pointcloud_merger
    pointcloud_merger.cpp
)

target_link_libraries(pointcloud_merger
    Eigen3::Eigen
    ${DORA_NODE_API_LIB}
)
