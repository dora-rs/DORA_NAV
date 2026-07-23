#ifndef BINARY_SERIALIZATION_H
#define BINARY_SERIALIZATION_H

/**
 * @file binary_serialization.h
 * @brief Binary point cloud deserialization for fast IPC between livox_driver and hdl_localization.
 *
 * Wire format (little-endian, packed):
 *   [0..7]   double   timestamp      (seconds, same origin as JSON version)
 *   [8..11]  uint32_t num_points
 *   [12..]   N × 16 bytes per point:
 *              float32 x
 *              float32 y
 *              float32 z
 *              float32 intensity
 *
 * Total size = 12 + N * 16  bytes.
 */

#include <cstdint>
#include <cstring>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace hdl_localization {

/**
 * @brief Deserialize a binary point cloud buffer produced by livox_driver.
 *
 * @param data      Pointer to raw byte buffer received from Dora.
 * @param len       Length of the buffer in bytes.
 * @param timestamp Output: timestamp extracted from the header (seconds).
 * @return Shared pointer to the parsed PCL cloud, or nullptr on error.
 */
inline pcl::PointCloud<pcl::PointXYZI>::Ptr
binaryToPointCloud(const char* data, size_t len, double& timestamp)
{
    // Minimum header size: 8 (timestamp) + 4 (count) = 12 bytes
    constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t);

    if (!data || len < HEADER_SIZE) {
        return nullptr;
    }

    // --- timestamp (8 bytes) ---
    std::memcpy(&timestamp, data, sizeof(double));
    const char* ptr = data + sizeof(double);

    // --- num_points (4 bytes) ---
    uint32_t num_points = 0;
    std::memcpy(&num_points, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    // Sanity check: remaining bytes must fit exactly N * 16
    constexpr size_t POINT_SIZE = 4 * sizeof(float); // x y z intensity
    if (len - HEADER_SIZE < static_cast<size_t>(num_points) * POINT_SIZE) {
        return nullptr;
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->points.resize(num_points);
    cloud->width    = num_points;
    cloud->height   = 1;
    cloud->is_dense = true;
    cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6); // µs for PCL

    for (uint32_t i = 0; i < num_points; ++i) {
        float vals[4];
        std::memcpy(vals, ptr, POINT_SIZE);
        ptr += POINT_SIZE;

        cloud->points[i].x         = vals[0];
        cloud->points[i].y         = vals[1];
        cloud->points[i].z         = vals[2];
        cloud->points[i].intensity = vals[3];
    }

    return cloud;
}

} // namespace hdl_localization

#endif // BINARY_SERIALIZATION_H
