cmake_minimum_required(VERSION 2.8.3)
project(local_planner_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(Eigen3 REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/dora_config.cmake)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${DORA_INCLUDE_DIR}
    ${DORA_OPERATOR_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${DORA_NAV_ROOT}/include
    ${DORA_NAV_ROOT}/control/include
)

add_executable(local_planner_node
    src/main.cpp
    src/local_planner.cpp
    src/obstacle_map.cpp
)

target_link_libraries(local_planner_node
    ${DORA_LIB_PATH}
    $<$<BOOL:${Python3_FOUND}>:Python3::Python>
    m
    rt
    dl
    pthread
)
