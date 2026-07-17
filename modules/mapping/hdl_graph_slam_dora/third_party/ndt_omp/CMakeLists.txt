cmake_minimum_required(VERSION 3.10)

# NDT OMP Static Library
add_library(ndt_omp_static STATIC
    src/pclomp/ndt_omp.cpp
    src/pclomp/gicp_omp.cpp
    src/pclomp/voxel_grid_covariance_omp.cpp
)

target_include_directories(ndt_omp_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ndt_omp_static
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

target_compile_options(ndt_omp_static PRIVATE
    -O3
    -march=native
)
