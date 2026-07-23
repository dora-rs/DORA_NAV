#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>
#include <iostream>

namespace hdl_graph_slam_dora {

/**
 * 解析点云二进制数据（与 hdl_localization_dora 相同格式）
 * Wire format (little-endian):
 *   [0..7]   double   timestamp (秒)
 *   [8..11]  uint32_t num_points
 *   [12..]   N × 16 bytes per point (x, y, z, intensity 各 4 字节 float)
 */
inline pcl::PointCloud<pcl::PointXYZI>::Ptr parsePointCloudBinary(
    const char* data,
    size_t len,
    double& timestamp
) {
    constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t);
    constexpr size_t POINT_SIZE = 4 * sizeof(float);

    if (!data || len < HEADER_SIZE) {
        std::cerr << "[BinaryParser] Buffer too small: " << len << " bytes" << std::endl;
        return nullptr;
    }

    // 解析时间戳
    std::memcpy(&timestamp, data, sizeof(double));
    const char* ptr = data + sizeof(double);

    // 解析点数量
    uint32_t num_points = 0;
    std::memcpy(&num_points, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    // 验证数据完整性
    size_t expected_size = HEADER_SIZE + num_points * POINT_SIZE;
    if (len != expected_size) {
        std::cerr << "[BinaryParser] Size mismatch. Expected: " << expected_size
                  << ", Got: " << len << std::endl;
        return nullptr;
    }

    // 创建点云
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->points.resize(num_points);
    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = false;

    // 解析点数据
    for (uint32_t i = 0; i < num_points; ++i) {
        float vals[4];
        std::memcpy(vals, ptr, POINT_SIZE);
        ptr += POINT_SIZE;

        cloud->points[i].x = vals[0];
        cloud->points[i].y = vals[1];
        cloud->points[i].z = vals[2];
        cloud->points[i].intensity = vals[3];
    }

    return cloud;
}

} // namespace hdl_graph_slam_dora
