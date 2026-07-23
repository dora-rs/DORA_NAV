#ifndef JSON_SERIALIZATION_H
#define JSON_SERIALIZATION_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>

namespace hdl_localization {

/**
 * @brief IMU data structure
 */
struct ImuData {
    double timestamp;
    Eigen::Vector3f acc;
    Eigen::Vector3f gyro;
};

/**
 * @brief JSON serialization utilities for converting between JSON and PCL/Eigen types
 */
class JsonSerializer {
public:
    /**
     * @brief Convert JSON point cloud to PCL PointCloud
     * @param json_data Input JSON data
     * @param timestamp Output timestamp extracted from header
     * @return PCL point cloud pointer
     */
    static pcl::PointCloud<pcl::PointXYZI>::Ptr
    jsonToPointCloud(const nlohmann::json& json_data, double& timestamp);

    /**
     * @brief Convert JSON IMU data to Eigen vectors
     * @param json_data Input JSON data
     * @param timestamp Output timestamp
     * @param acc Output linear acceleration
     * @param gyro Output angular velocity
     * @return true if successful, false otherwise
     */
    static bool jsonToImu(const nlohmann::json& json_data,
                          double& timestamp,
                          Eigen::Vector3f& acc,
                          Eigen::Vector3f& gyro);

    /**
     * @brief Convert pose matrix to JSON
     * @param pose 4x4 transformation matrix
     * @param timestamp Timestamp
     * @param seq Sequence number
     * @return JSON representation of pose
     */
    static nlohmann::json poseToJson(const Eigen::Matrix4f& pose,
                                      double timestamp,
                                      uint32_t seq);

    /**
     * @brief Convert pose matrix to JSON with velocity
     * @param pose 4x4 transformation matrix
     * @param linear_vel Linear velocity
     * @param angular_vel Angular velocity
     * @param timestamp Timestamp
     * @param seq Sequence number
     * @return JSON representation of pose with twist
     */
    static nlohmann::json poseToJson(const Eigen::Matrix4f& pose,
                                      const Eigen::Vector3f& linear_vel,
                                      const Eigen::Vector3f& angular_vel,
                                      double timestamp,
                                      uint32_t seq);

    /**
     * @brief Convert PCL point cloud to JSON
     * @param cloud Input point cloud
     * @param timestamp Timestamp
     * @return JSON representation of point cloud
     */
    static nlohmann::json pointCloudToJson(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
                                            double timestamp);
};

} // namespace hdl_localization

#endif // JSON_SERIALIZATION_H
