#include "json_serialization.h"
#include <iostream>
#include <boost/make_shared.hpp>

namespace hdl_localization {

pcl::PointCloud<pcl::PointXYZI>::Ptr
JsonSerializer::jsonToPointCloud(const nlohmann::json& json_data, double& timestamp) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    try {
        // Extract header
        if (!json_data.contains("header") || !json_data.contains("points")) {
            std::cerr << "Invalid point cloud JSON: missing header or points" << std::endl;
            return nullptr;
        }

        // Livox driver now outputs timestamp in seconds
        timestamp = json_data["header"]["timestamp"];

        if (json_data["header"].contains("frame_id")) {
            cloud->header.frame_id = json_data["header"]["frame_id"];
        }

        // Convert timestamp to microseconds for PCL header
        cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);

        // Extract points
        const auto& points_array = json_data["points"];
        cloud->points.reserve(points_array.size());

        for (const auto& pt : points_array) {
            pcl::PointXYZI point;
            point.x = pt["x"];
            point.y = pt["y"];
            point.z = pt["z"];
            point.intensity = pt["intensity"];
            cloud->points.push_back(point);
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = json_data.value("is_dense", true);

        return cloud;

    } catch (const std::exception& e) {
        std::cerr << "Error parsing point cloud JSON: " << e.what() << std::endl;
        return nullptr;
    }
}

bool JsonSerializer::jsonToImu(const nlohmann::json& json_data,
                                double& timestamp,
                                Eigen::Vector3f& acc,
                                Eigen::Vector3f& gyro) {
    try {
        if (!json_data.contains("header") ||
            !json_data.contains("linear_acceleration") ||
            !json_data.contains("angular_velocity")) {
            std::cerr << "Invalid IMU JSON: missing required fields" << std::endl;
            return false;
        }

        // Livox driver now outputs timestamp in seconds
        timestamp = json_data["header"]["timestamp"];

        // Livox IMU outputs acceleration in g units, convert to m/s²
        // This is CRITICAL: without this conversion, acceleration is underestimated by 9.8x
        // which causes severe drift in IMU-based pose estimation
        const float G = 9.80665f;  // Standard gravity in m/s²
        acc.x() = json_data["linear_acceleration"]["x"].get<float>() * G;
        acc.y() = json_data["linear_acceleration"]["y"].get<float>() * G;
        acc.z() = json_data["linear_acceleration"]["z"].get<float>() * G;

        gyro.x() = json_data["angular_velocity"]["x"];
        gyro.y() = json_data["angular_velocity"]["y"];
        gyro.z() = json_data["angular_velocity"]["z"];

        return true;

    } catch (const std::exception& e) {
        std::cerr << "Error parsing IMU JSON: " << e.what() << std::endl;
        return false;
    }
}

nlohmann::json JsonSerializer::poseToJson(const Eigen::Matrix4f& pose,
                                           double timestamp,
                                           uint32_t seq) {
    nlohmann::json json_data;

    // Header
    json_data["header"]["frame_id"] = "map";
    json_data["header"]["timestamp"] = timestamp;
    json_data["header"]["seq"] = seq;

    // Extract position and orientation
    Eigen::Vector3f position = pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation = pose.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    quat.normalize();

    // Position
    json_data["pose"]["position"]["x"] = position.x();
    json_data["pose"]["position"]["y"] = position.y();
    json_data["pose"]["position"]["z"] = position.z();

    // Orientation (quaternion)
    json_data["pose"]["orientation"]["w"] = quat.w();
    json_data["pose"]["orientation"]["x"] = quat.x();
    json_data["pose"]["orientation"]["y"] = quat.y();
    json_data["pose"]["orientation"]["z"] = quat.z();

    // Twist removed for simplification - only pose data is published
    // json_data["twist"]["linear"]["x"] = 0.0;
    // json_data["twist"]["linear"]["y"] = 0.0;
    // json_data["twist"]["linear"]["z"] = 0.0;
    // json_data["twist"]["angular"]["x"] = 0.0;
    // json_data["twist"]["angular"]["y"] = 0.0;
    // json_data["twist"]["angular"]["z"] = 0.0;

    return json_data;
}

nlohmann::json JsonSerializer::poseToJson(const Eigen::Matrix4f& pose,
                                           const Eigen::Vector3f& linear_vel,
                                           const Eigen::Vector3f& angular_vel,
                                           double timestamp,
                                           uint32_t seq) {
    nlohmann::json json_data;

    // Header
    json_data["header"]["frame_id"] = "map";
    json_data["header"]["timestamp"] = timestamp;
    json_data["header"]["seq"] = seq;

    // Extract position and orientation
    Eigen::Vector3f position = pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation = pose.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    quat.normalize();

    // Position
    json_data["pose"]["position"]["x"] = position.x();
    json_data["pose"]["position"]["y"] = position.y();
    json_data["pose"]["position"]["z"] = position.z();

    // Orientation (quaternion)
    json_data["pose"]["orientation"]["w"] = quat.w();
    json_data["pose"]["orientation"]["x"] = quat.x();
    json_data["pose"]["orientation"]["y"] = quat.y();
    json_data["pose"]["orientation"]["z"] = quat.z();

    // Twist (velocity) - commented out for simplification
    // json_data["twist"]["linear"]["x"] = linear_vel.x();
    // json_data["twist"]["linear"]["y"] = linear_vel.y();
    // json_data["twist"]["linear"]["z"] = linear_vel.z();
    // json_data["twist"]["angular"]["x"] = angular_vel.x();
    // json_data["twist"]["angular"]["y"] = angular_vel.y();
    // json_data["twist"]["angular"]["z"] = angular_vel.z();

    return json_data;
}

nlohmann::json JsonSerializer::pointCloudToJson(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
                                                 double timestamp) {
    nlohmann::json json_data;

    // Header
    json_data["header"]["frame_id"] = cloud->header.frame_id.empty() ? "map" : cloud->header.frame_id;
    json_data["header"]["timestamp"] = timestamp;
    json_data["header"]["seq"] = cloud->header.seq;

    // Point cloud metadata
    json_data["width"] = cloud->width;
    json_data["height"] = cloud->height;
    json_data["is_dense"] = cloud->is_dense;

    // Points array
    json_data["points"] = nlohmann::json::array();

    for (const auto& pt : cloud->points) {
        nlohmann::json point;
        point["x"] = pt.x;
        point["y"] = pt.y;
        point["z"] = pt.z;
        point["intensity"] = pt.intensity;
        json_data["points"].push_back(point);
    }

    return json_data;
}

} // namespace hdl_localization
