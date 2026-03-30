cmake_minimum_required(VERSION 3.10)
project(multi_lidar_calibration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

#========================
#  dora
#========================
if(NOT DORA_INCLUDE_DIR)
  set(DORA_INCLUDE_DIR $ENV{HOME}/dora/apis/c/node; $ENV{HOME}/dora/apis/c/operator)
endif()
if(NOT DORA_NODE_API_LIB)
  set(DORA_NODE_API_LIB $ENV{HOME}/dora/target/release/libdora_node_api_c.a)
endif()


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration io)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${DORA_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(multi_lidar_calibrator
    src/multi_lidar_calibrator.cpp
    src/multi_lidar_calibrator_node.cpp
)

target_link_libraries(multi_lidar_calibrator
    ${PCL_LIBRARIES}
    ${DORA_NODE_API_LIB}
)
