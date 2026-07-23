cmake_minimum_required(VERSION 3.10)

# Fast GICP Static Library (CPU only, no CUDA)
# Explicitly list source files to avoid experimental code with missing dependencies
add_library(fast_gicp_static STATIC
    src/fast_gicp/gicp/lsq_registration.cpp
    src/fast_gicp/gicp/fast_gicp.cpp
    src/fast_gicp/gicp/fast_gicp_st.cpp
    src/fast_gicp/gicp/fast_vgicp.cpp
)

target_include_directories(fast_gicp_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus
)

target_link_libraries(fast_gicp_static
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

target_compile_options(fast_gicp_static PRIVATE
    -O3
    # -march=native  # 注释掉以避免指令集不兼容问题
)
