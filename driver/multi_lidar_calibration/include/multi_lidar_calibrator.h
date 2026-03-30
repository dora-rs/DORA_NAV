#ifndef PROJECT_MULTI_LIDAR_CALIBRATOR_H
#define PROJECT_MULTI_LIDAR_CALIBRATOR_H

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <fstream>
#include <iostream>
#include <cstdint>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#define __APP_NAME__ "multi_lidar_calibrator"

// ---------------------------------------------------------------------------
// Binary point cloud format shared with lidar driver nodes:
//   bytes  0- 3 : seq       (uint32_t)
//   bytes  4- 7 : padding
//   bytes  8-15 : timestamp (uint64_t, microseconds)
//   bytes 16+   : N × [x, y, z, intensity]  (4 × float32, 16 bytes/point)
// ---------------------------------------------------------------------------

class MultiLidarCalibratorApp
{
public:
    MultiLidarCalibratorApp();

    // Entry point – owns the dora event loop
    void Run(void *dora_context);

private:
    // ---- dora context ----
    void *dora_context_;

    // ---- point clouds ----
    typedef pcl::PointXYZ PointT;

    pcl::PointCloud<PointT>::Ptr in_parent_cloud_;
    pcl::PointCloud<PointT>::Ptr in_child_cloud_;
    pcl::PointCloud<PointT>::Ptr in_child_filtered_cloud_;

    // timestamps of the latest received frames (microseconds)
    uint64_t parent_timestamp_us_;
    uint64_t child_timestamp_us_;

    std::string parent_frame_;
    std::string child_frame_;

    // ---- calibration state ----
    Eigen::Matrix4f current_guess_;

    // ---- parameters ----
    double voxel_size_;
    double ndt_epsilon_;
    double ndt_step_size_;
    double ndt_resolution_;
    int    ndt_iterations_;

    // initial pose map: topic_name → [x, y, z, yaw, pitch, roll]
    std::map<std::string, std::vector<double>> transfer_map_;

    int         child_topic_num_;
    std::string points_parent_topic_str_;
    std::string points_child_topic_str_;

    // ---- helpers ----

    /**
     * Parse a raw dora binary buffer into a PCL point cloud.
     * @param data      pointer to the payload bytes
     * @param len       payload length in bytes
     * @param timestamp_us  (out) timestamp parsed from the header
     * @param cloud     (out) destination cloud
     * @return true on success
     */
    bool ParsePointCloud(const uint8_t *data, size_t len,
                         uint64_t &timestamp_us,
                         pcl::PointCloud<PointT>::Ptr &cloud);

    /**
     * Voxel-grid downsample.
     */
    void DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                         pcl::PointCloud<PointT>::Ptr       out_cloud_ptr,
                         double in_leaf_size);

    /**
     * Serialize a PCL cloud to the shared binary format and send via dora.
     * Output id: "pointcloud_calibrated"
     */
    void PublishCloud(pcl::PointCloud<PointT>::ConstPtr cloud_ptr,
                      uint64_t timestamp_us);

    /**
     * Run one NDT iteration using the currently buffered clouds.
     */
    void PerformNdtOptimize();

    /**
     * Load parameters from config file and set NDT defaults.
     * @param init_file_path  path to child_topic_list style config
     */
    void Initialize(const std::string &init_file_path);
};

#endif // PROJECT_MULTI_LIDAR_CALIBRATOR_H
