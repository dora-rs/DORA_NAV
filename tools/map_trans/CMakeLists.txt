cmake_minimum_required(VERSION 3.10)
project(map_trans)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.8 REQUIRED COMPONENTS common io filters)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd2pgm
    src/main.cpp
    src/config_loader.cpp
    src/pcd_reader.cpp
    src/grid_map.cpp
    src/pgm_writer.cpp
)

target_link_libraries(pcd2pgm ${PCL_LIBRARIES})

target_compile_options(pcd2pgm PRIVATE -Wall -Wextra)
