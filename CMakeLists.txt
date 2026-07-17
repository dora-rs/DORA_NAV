cmake_minimum_required(VERSION 3.10)
project(NavigationFramework)

# ── 全局编译设置 ──────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type: Debug/Release/RelWithDebInfo")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# ── dora SDK（唯一入口，所有子模块直接使用这两个变量）────────
set(DORA_ROOT ${CMAKE_SOURCE_DIR}/third_party/dora)
set(DORA_INCLUDE_DIR ${DORA_ROOT}/include)
set(DORA_NODE_API_LIB ${DORA_ROOT}/lib/libdora_node_api_c.a)

if(NOT EXISTS ${DORA_NODE_API_LIB})
    message(FATAL_ERROR "dora SDK not found at ${DORA_ROOT}. "
        "Please ensure third_party/dora/lib/libdora_node_api_c.a exists.")
endif()
 
# 如果PCL库报错就需要指定 Qt 的路径（如果需要使用 Qt，且 Qt 安装在 Conda 环境中）
# set(QT_HOST_PATH $ENV{CONDA_PREFIX})
# ── 公共第三方依赖（子模块直接使用查找结果，无需重复 find）───
find_package(PCL 1.7 QUIET COMPONENTS common io filters kdtree registration)
if(PCL_FOUND)
    message(STATUS "PCL ${PCL_VERSION} found")
endif()

find_package(Eigen3 QUIET)
find_package(OpenMP QUIET)
find_package(Boost QUIET COMPONENTS system filesystem)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp QUIET)
find_package(PkgConfig QUIET)
find_package(Threads REQUIRED)

# ── 子模块编译开关 ────────────────────────────────────────────
option(BUILD_DRIVER             "Build driver node"                 ON)
option(BUILD_CHASSIS            "Build chassis node"                ON)
option(BUILD_LOCALIZATION       "Build localization node"           ON)
option(BUILD_MAPPING            "Build mapping node"                ON)
option(BUILD_GLOBAL_PLANNER     "Build global planner node"         ON)
option(BUILD_LOCAL_PLANNER      "Build local planner node"          ON)
option(BUILD_GOAL_PUBLISHER     "Build goal publisher node"         ON)
option(BUILD_PERCEPTION         "Build perception node"             ON)
option(BUILD_VISUALIZATION      "Build Rerun visualization node"    ON)

# ── 子模块 ────────────────────────────────────────────────────
if(BUILD_DRIVER)
    add_subdirectory(modules/drivers/livox_driver)
    add_subdirectory(modules/drivers/rslidar_sdk)
endif()

if(BUILD_CHASSIS)
    # add_subdirectory(modules/chassis/dora_mickrobot)
    add_subdirectory(modules/chassis/ranger_dora_node)
endif()

if(BUILD_LOCALIZATION)
    add_subdirectory(modules/localization/hdl_localization_dora)
endif()

if(BUILD_MAPPING)
    # add_subdirectory(modules/mapping/ndt_mapping_dora)
    add_subdirectory(modules/mapping/lightning_lm_mapping)
    add_subdirectory(modules/mapping/hdl_graph_slam_dora)
endif()

if(BUILD_GLOBAL_PLANNER)
    add_subdirectory(modules/planner/global_planner/astar_planner_dora)
endif()

if(BUILD_LOCAL_PLANNER)
    add_subdirectory(modules/planner/local_planner/pure_pursuit_dora)
    # add_subdirectory(modules/planner/local_planner/dwa_planner_dora)
endif()

if(BUILD_GOAL_PUBLISHER)
    add_subdirectory(modules/interface/goal_publisher)
endif()

if(BUILD_VISUALIZATION)
    add_subdirectory(modules/visualization/rerun_visualizer)
    add_subdirectory(modules/visualization/rerun_mapping)
    add_subdirectory(modules/visualization/rerun_lidar)
endif()

# ── 编译摘要 ──────────────────────────────────────────────────
message(STATUS "")
message(STATUS "========== NavigationFramework Build Configuration ==========")
message(STATUS "Build type      : ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard    : ${CMAKE_CXX_STANDARD}")
message(STATUS "dora SDK        : ${DORA_ROOT}")
message(STATUS "---")
message(STATUS "driver          : ${BUILD_DRIVER}")
message(STATUS "chassis         : ${BUILD_CHASSIS}")
message(STATUS "localization    : ${BUILD_LOCALIZATION}")
message(STATUS "mapping         : ${BUILD_MAPPING}")
message(STATUS "global_planner  : ${BUILD_GLOBAL_PLANNER}")
message(STATUS "local_planner   : ${BUILD_LOCAL_PLANNER}")
message(STATUS "goal_publisher  : ${BUILD_GOAL_PUBLISHER}")
message(STATUS "perception      : ${BUILD_PERCEPTION}")
message(STATUS "visualization   : ${BUILD_VISUALIZATION}")
message(STATUS "=============================================================")
message(STATUS "")
