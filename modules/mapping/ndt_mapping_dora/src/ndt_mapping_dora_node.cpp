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

#include "ndt_mapping/ndt_mapper.hpp"
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <vector>

// 6DOF 位姿线格式：{double x, y, z, roll, pitch, yaw}
typedef struct pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
}Pose_t;

Pose_t cur_pose__;

void Send_PoseData(void *dora_context, ndt_mapping::Pose& cur_pose)
{
    cur_pose__.x     = cur_pose.x;
    cur_pose__.y     = cur_pose.y;
    cur_pose__.z     = cur_pose.z;
    cur_pose__.roll  = cur_pose.roll;
    cur_pose__.pitch = cur_pose.pitch;
    cur_pose__.yaw   = cur_pose.yaw;
    /* 发送估计位姿 */
    std::string output_id = "pose_data";
    size_t id_len = output_id.length();
    char *output_data = reinterpret_cast<char*>(&cur_pose__);
    size_t output_data_len = sizeof(cur_pose__);
    dora_send_output(dora_context, &output_id[0], id_len, output_data, output_data_len);
}

// 发送本帧增量点云块（紧凑 x,y,z float 流）。接收端按 len/(3*sizeof(float)) 求点数。
void Send_MappingData(void *dora_context, pcl::PointCloud<pcl::PointXYZI> &increment)
{
    std::string output_id = "mapping_data";
    size_t id_len = output_id.length();

    std::vector<float> coordinates;
    coordinates.reserve(increment.size() * 3);
    for ( const auto& point : increment.points)
    {
        coordinates.push_back(point.x);
        coordinates.push_back(point.y);
        coordinates.push_back(point.z);
    }

    dora_send_output(dora_context, &output_id[0], id_len, reinterpret_cast<char*>(coordinates.data()),
                     coordinates.size() * sizeof(float));
}


int mapping(void *dora_context, ndt_mapping::NDTMapper &mapper){
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;
    }

    printf("[c node] dora context initialized\n");

    while(true){
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);

            // livox 线格式: [0..7] double ts | [8..11] uint32 n | [12..] n×16 float(x,y,z,intensity)
            if ( strncmp(data_id,"pointcloud",data_id_len) == 0)
            {
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

                pcl::PointCloud<pcl::PointXYZI> pointcloud;
                pointcloud.header.stamp = static_cast<std::uint64_t>(ts * 1e6); // us
                pointcloud.header.frame_id = "lidar";
                pointcloud.width = n;
                pointcloud.height = 1;
                pointcloud.is_dense = true;
                pointcloud.points.resize(n);

                const char* ptr = data + HEADER_SIZE;
                for (uint32_t i = 0; i < n; ++i, ptr += POINT_SIZE) {
                    float vals[4];
                    std::memcpy(vals, ptr, POINT_SIZE);
                    pointcloud.points[i].x         = vals[0];
                    pointcloud.points[i].y         = vals[1];
                    pointcloud.points[i].z         = vals[2];
                    pointcloud.points[i].intensity = vals[3];
                }

                pointcloud_ptr = pointcloud.makeShared();
                mapper.points_callback(pointcloud_ptr);
                pointcloud.clear();
                pointcloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);

                /********************* 发送 dora 数据 *********************/
                // 仅当本帧有新拼入的增量块时才发送地图增量，避免随地图增长全量重传
                if (mapper.has_new_scan) {
                    Send_MappingData(dora_context, mapper.last_increment);
                }
                Send_PoseData(dora_context, mapper.current_pose);
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
            break;
        }
        else if (ty == DoraEventType_Error)
        {
            printf("[c node] received error event (e.g., Ctrl+C)\n");
            break;
        }
        else if (ty == DoraEventType_InputClosed)
        {
            printf("[c node] received input closed event\n");
            break;
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
            break;
        }

        free_dora_event(event);
    }

    printf("[c node] event loop exited normally\n");
    return 0;
}


int main(int argc, char* argv[])
{
    printf("dora ndt mapping node\n");

    void *dora_context = init_dora_context_from_env();

    std::string config_path = "../modules/mapping/ndt_mapping_dora/config/ndt_mapping_config.yml";

    ndt_mapping::NDTMapper mapper(config_path);

    mapping(dora_context, mapper);

    // 无论事件循环如何退出，都保存地图
    printf("[c node] saving map before exit...\n");
    mapper.saveMap();

    free_dora_context(dora_context);

    printf("[c node] finished successfully\n");

    return 0;
}
