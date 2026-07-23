project(rslidar_dora_node)

#========================
# Point Type - Fixed to XYZI (unified with Livox driver)
#========================
add_definitions(-DPOINT_TYPE_XYZI)

#========================
# Driver Options
#========================
option(ENABLE_DIFOP_PARSE "Enable parsing DIFOP Packet" ON)
if(ENABLE_DIFOP_PARSE)
  add_definitions(-DENABLE_DIFOP_PARSE)
endif()

option(ENABLE_MODIFY_RECVBUF "Enable modify size of RECVBUF" ON)
if(ENABLE_MODIFY_RECVBUF)
  add_definitions(-DENABLE_MODIFY_RECVBUF)
endif()

option(ENABLE_IMU_DATA_PARSE "Enable IMU data parse (only for RSAIRY, RSE1)" ON)
if(ENABLE_IMU_DATA_PARSE)
  add_definitions(-DENABLE_IMU_DATA_PARSE)
endif()

#========================
# Include Directories
#========================
include_directories(
    src
    ${DORA_INCLUDE_DIR}
)

#========================
# rs_driver Submodule
#========================
add_subdirectory(src/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})

#========================
# Build Executable
#========================
add_executable(${PROJECT_NAME}
    node/rslidar_dora_node.cpp
    src/manager/node_manager.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${rs_driver_LIBRARIES}
    nlohmann_json::nlohmann_json
    yaml-cpp
    ${DORA_NODE_API_LIB}
)

message(STATUS "rslidar_dora_node: Point type XYZI, IMU parsing ${ENABLE_IMU_DATA_PARSE}")
