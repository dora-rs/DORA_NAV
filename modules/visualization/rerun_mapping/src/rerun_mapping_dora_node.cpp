extern "C"{
#include "node_api.h"
}

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <cstdint>
#include <cstring>

#include "rerun_mapping/rerun_mapper.hpp"
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>

// 6DOF 位姿结构
typedef struct pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
} Pose_t;

int rerun_mapping_loop(void *dora_context, rerun_mapping::RerunMapper &mapper) {
    if (dora_context == NULL) {
        fprintf(stderr, "[rerun_mapping] failed to init dora context\n");
        return -1;
    }

    printf("[rerun_mapping] dora context initialized\n");

    while (true) {
        void *event = dora_next_event(dora_context);
        if (event == NULL) {
            printf("[rerun_mapping] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input) {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);

            // 处理位姿数据
            if (strncmp(data_id, "pose_data", data_id_len) == 0) {
                if (data_len == sizeof(Pose_t)) {
                    Pose_t pose_data;
                    std::memcpy(&pose_data, data, sizeof(Pose_t));

                    rerun_mapping::Pose pose(
                        pose_data.x, pose_data.y, pose_data.z,
                        pose_data.roll, pose_data.pitch, pose_data.yaw
                    );

                    mapper.onPoseData(pose);
                }
            }
            // 处理增量地图数据
            else if (strncmp(data_id, "mapping_data", data_id_len) == 0) {
                // mapping_data 格式: float数组 [x1, y1, z1, x2, y2, z2, ...]
                size_t num_floats = data_len / sizeof(float);
                if (num_floats % 3 == 0 && num_floats > 0) {
                    const float* float_data = reinterpret_cast<const float*>(data);
                    mapper.onMappingData(float_data, num_floats);
                }
            }
            // 处理原始点云数据
            else if (strncmp(data_id, "pointcloud", data_id_len) == 0) {
                // pointcloud 格式: [0..7] double ts | [8..11] uint32 n | [12..] n×16 float(x,y,z,intensity)
                // 用于单帧点云可视化
                constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t); // 12
                constexpr size_t POINT_SIZE  = 4 * sizeof(float);                 // 16

                if (data_len < HEADER_SIZE) {
                    free_dora_event(event);
                    continue;
                }

                double ts;
                std::memcpy(&ts, data, sizeof(double));
                uint32_t n = 0;
                std::memcpy(&n, data + sizeof(double), sizeof(uint32_t));

                // 校验剩余字节恰好容纳 n 个点
                if (data_len - HEADER_SIZE < static_cast<size_t>(n) * POINT_SIZE) {
                    free_dora_event(event);
                    continue;
                }

                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
                pointcloud->header.stamp = static_cast<std::uint64_t>(ts * 1e6); // us
                pointcloud->header.frame_id = "lidar";
                pointcloud->width = n;
                pointcloud->height = 1;
                pointcloud->is_dense = true;
                pointcloud->points.resize(n);

                const char* ptr = data + HEADER_SIZE;
                for (uint32_t i = 0; i < n; ++i, ptr += POINT_SIZE) {
                    float vals[4];
                    std::memcpy(vals, ptr, POINT_SIZE);
                    pointcloud->points[i].x         = vals[0];
                    pointcloud->points[i].y         = vals[1];
                    pointcloud->points[i].z         = vals[2];
                    pointcloud->points[i].intensity = vals[3];
                }

                mapper.onPointCloud(pointcloud);
            }
            // 新增：完整地图话题（Lightning-LM 回环后发送）
            else if (strncmp(data_id, "full_map", data_id_len) == 0) {
                // full_map 格式: 与 pointcloud 相同，但语义是"完整地图替换"
                constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t);
                constexpr size_t POINT_SIZE  = 4 * sizeof(float);

                if (data_len < HEADER_SIZE) {
                    free_dora_event(event);
                    continue;
                }

                double ts;
                std::memcpy(&ts, data, sizeof(double));
                uint32_t n = 0;
                std::memcpy(&n, data + sizeof(double), sizeof(uint32_t));

                if (data_len - HEADER_SIZE < static_cast<size_t>(n) * POINT_SIZE) {
                    free_dora_event(event);
                    continue;
                }

                pcl::PointCloud<pcl::PointXYZI>::Ptr full_map(new pcl::PointCloud<pcl::PointXYZI>);
                full_map->header.stamp = static_cast<std::uint64_t>(ts * 1e6);
                full_map->header.frame_id = "map";
                full_map->width = n;
                full_map->height = 1;
                full_map->is_dense = true;
                full_map->points.resize(n);

                const char* ptr = data + HEADER_SIZE;
                for (uint32_t i = 0; i < n; ++i, ptr += POINT_SIZE) {
                    float vals[4];
                    std::memcpy(vals, ptr, POINT_SIZE);
                    full_map->points[i].x         = vals[0];
                    full_map->points[i].y         = vals[1];
                    full_map->points[i].z         = vals[2];
                    full_map->points[i].intensity = vals[3];
                }

                printf("[rerun_mapping] Received full_map with %u points\n", n);
                mapper.onFullMap(full_map);
            }
        }
        else if (ty == DoraEventType_Stop) {
            printf("[rerun_mapping] received stop event\n");
            break;
        }
        else {
            printf("[rerun_mapping] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }

    return 0;
}

int main(int argc, char* argv[]) {
    printf("[rerun_mapping] Starting rerun mapping visualization node\n");

    void *dora_context = init_dora_context_from_env();

    rerun_mapping::RerunMapper mapper;

    int result = rerun_mapping_loop(dora_context, mapper);

    free_dora_context(dora_context);

    printf("[rerun_mapping] finished %s\n", result == 0 ? "successfully" : "with errors");

    return result;
}
