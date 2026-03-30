cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

project(livox_dora_driver VERSION 1.0.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 标准编译选项，无需额外兼容配置
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_CXX_FLAGS}")

set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "Current dir stored in variable: ${CURRENT_DIR}")

find_package(Boost 1.54 REQUIRED COMPONENTS system thread chrono)
find_package(PkgConfig)

pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif (APR_FOUND)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找系统全局安装的Livox SDK静态库（make install自动部署到/usr/lib/）
find_library(LIVOX_LIDAR_SDK_LIBRARY  
  #NAMES liblivox_sdk_static.a
  NAMES liblivox_lidar_sdk_static.a
  PATHS /usr/local/lib/
  REQUIRED
)
find_package(PkgConfig)

add_executable(${PROJECT_NAME}_node "")

target_compile_options(${PROJECT_NAME}_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>)

# 保留项目所有原有源文件，无增删
target_sources(${PROJECT_NAME}_node
  PRIVATE
  src/lds.cpp
  src/lds_lidar.cpp
  src/lddc_dora.cpp
  src/livox_dora_driver2.cpp
  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp
  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp
  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

if(NOT DORA_INCLUDE_DIR)
  set(DORA_INCLUDE_DIR $ENV{HOME}/dora/apis/c/node; $ENV{HOME}/dora/apis/c/operator)
endif()
if(NOT DORA_NODE_API_LIB)
  set(DORA_NODE_API_LIB $ENV{HOME}/dora/target/release/libdora_node_api_c.a)
endif()


# 头文件目录：系统全局路径（/usr/include/已包含Livox所有头文件）+ 项目本地路径
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  /usr/local/include/  # 核心：系统全局头文件路径，包含官方Livox SDK所有头文件
  3rdparty
  src
)

# 链接逻辑：链接系统全局Livox库+其他依赖，Boost变量已修正为LIBRARIES
target_link_libraries(${PROJECT_NAME}_node
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${APR_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${DORA_NODE_API_LIB}
  m 
  rt 
  dl 
  pthread 
  pcap
)
 

# Add Livox SDK link
#target_link_libraries(livox_dora_driver_node pcap livox_lidar_sdk_shared)

