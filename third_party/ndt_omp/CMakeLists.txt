cmake_minimum_required(VERSION 3.10)
project(ndt_omp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)

add_library(ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

target_include_directories(ndt_omp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(ndt_omp PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(ndt_omp PUBLIC ${PCL_LIBRARIES})

if (OpenMP_CXX_FOUND)
  target_link_libraries(ndt_omp PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(align apps/align.cpp)
target_link_libraries(align PRIVATE ndt_omp)

# install targets and headers
install(TARGETS ndt_omp align
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)
