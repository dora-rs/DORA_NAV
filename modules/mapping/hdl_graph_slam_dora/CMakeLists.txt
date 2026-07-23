cmake_minimum_required(VERSION 3.10)
project(hdl_graph_slam_dora)

# ============================================================
# 1. 包含目录
# ============================================================
include_directories(
    include
    hdl_core/include
    third_party/ndt_omp/include
    third_party/fast_gicp/include
    third_party/fast_gicp/thirdparty/Eigen
    third_party/fast_gicp/thirdparty/Sophus
    ${DORA_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# ============================================================
# 2. 查找 g2o
# ============================================================
find_package(g2o REQUIRED)
if(g2o_FOUND)
    message(STATUS "g2o found")
    # 设置 g2o 库目标（使用 g2o 导出的 CMake 目标）
    set(G2O_LIBRARIES
        g2o::core
        g2o::stuff
        g2o::types_slam3d
        g2o::types_slam3d_addons
        g2o::solver_pcg
        g2o::solver_dense
        g2o::solver_eigen
    )
else()
    message(FATAL_ERROR "g2o not found. Please install g2o library.")
endif()

# ============================================================
# 3. 编译子模块
# ============================================================
add_subdirectory(third_party/ndt_omp)
add_subdirectory(third_party/fast_gicp)

# ============================================================
# 4. 编译 hdl_graph_slam 核心库
# ============================================================
file(GLOB HDL_CORE_SOURCES
    hdl_core/src/hdl_graph_slam/*.cpp
    hdl_core/src/g2o/*.cpp
)

add_library(hdl_graph_slam_core STATIC ${HDL_CORE_SOURCES})

target_link_libraries(hdl_graph_slam_core
    ${PCL_LIBRARIES}
    ${g2o_LIBRARIES}
    ndt_omp_static
    fast_gicp_static
    OpenMP::OpenMP_CXX
)

# ============================================================
# 5. 编译 Dora 节点
# ============================================================
add_executable(hdl_graph_slam_dora_node
    src/hdl_graph_slam_dora_node.cpp
    src/config_manager.cpp
    src/binary_serialization.cpp
    src/imu_parser.cpp
    src/prefiltering_module.cpp
    src/scan_matching_odometry_module.cpp
    src/graph_slam_module.cpp
    src/map_saver.cpp
)

target_link_libraries(hdl_graph_slam_dora_node
    hdl_graph_slam_core
    ndt_omp_static
    fast_gicp_static
    ${DORA_NODE_API_LIB}
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
    nlohmann_json::nlohmann_json
    Threads::Threads
    OpenMP::OpenMP_CXX
)

target_compile_options(hdl_graph_slam_dora_node PRIVATE
    -O3
    # -march=native  # 注释掉以避免指令集不兼容问题
    -Wall
)
