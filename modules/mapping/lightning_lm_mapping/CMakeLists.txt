project(lightning_lm_mapping)

# ── PCL 配置 ──────────────────────────────────────────────────
add_definitions("-DPCL_NO_PRECOMPILE")  # PCL 自定义点类型支持

# ── 查找依赖包 ────────────────────────────────────────────────
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)

# ── 包含目录 ──────────────────────────────────────────────────
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${DORA_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

# ── 核心库：收集所有核心算法源文件 ─────────────────────────────
file(GLOB_RECURSE CORE_SOURCES
    src/common/*.cc
    src/core/lio/*.cc
    src/core/loop_closing/*.cc
    src/core/g2p5/*.cc
    src/core/ivox3d/*.cc
    src/core/maps/*.cc
    src/core/miao/core/*.cc
    src/core/system/*.cc
    src/utils/*.cc
)

# 创建核心算法静态库
add_library(lightning_core STATIC ${CORE_SOURCES})

target_include_directories(lightning_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/miao   # miao库使用"core/xxx"路径引用
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lightning_core
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    glog
    gflags
    Threads::Threads
)

# ── DORA 节点可执行文件 ───────────────────────────────────────
add_executable(lightning_slam_node
    src/lightning_slam_node.cpp
)

target_include_directories(lightning_slam_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${DORA_INCLUDE_DIR}
)

target_link_libraries(lightning_slam_node
    lightning_core
    ${DORA_NODE_API_LIB}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    nlohmann_json::nlohmann_json
    glog
    gflags
    Threads::Threads
)

message(STATUS "Lightning-LM Mapping: configured")

