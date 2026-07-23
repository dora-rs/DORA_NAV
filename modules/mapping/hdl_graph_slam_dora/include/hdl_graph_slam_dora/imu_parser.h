#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <iostream>

namespace hdl_graph_slam_dora {

struct ImuData {
    double timestamp;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
    Eigen::Quaterniond orientation;
};

/**
 * 解析 IMU JSON 数据
 * 支持两种格式：
 * 格式1（直接timestamp）：
 * {
 *   "timestamp": 1234567890.123456,
 *   "linear_acceleration": {"x": 0.01, "y": -0.02, "z": 9.81},
 *   "angular_velocity": {"x": 0.001, "y": -0.002, "z": 0.003},
 *   "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
 * }
 * 格式2（header.timestamp，来自MID360）：
 * {
 *   "header": {"timestamp": 1234567890.123456, "frame_id": "..."},
 *   "linear_acceleration": {"x": 0.01, "y": -0.02, "z": 9.81},
 *   "angular_velocity": {"x": 0.001, "y": -0.002, "z": 0.003}
 * }
 */
inline bool parseImuJson(const char* data, size_t len, ImuData& imu_data) {
    try {
        nlohmann::json j = nlohmann::json::parse(data, data + len);

        // 尝试从顶层读取timestamp（格式1）
        if (j.contains("timestamp")) {
            imu_data.timestamp = j["timestamp"].get<double>();
        }
        // 否则尝试从header.timestamp读取（格式2，MID360格式）
        else if (j.contains("header") && j["header"].contains("timestamp")) {
            imu_data.timestamp = j["header"]["timestamp"].get<double>();
        }
        else {
            std::cerr << "[ImuParser] Missing 'timestamp' field (neither top-level nor in header)" << std::endl;
            return false;
        }


        if (j.contains("linear_acceleration")) {
            auto acc = j["linear_acceleration"];
            imu_data.linear_acceleration.x() = acc["x"].get<double>();
            imu_data.linear_acceleration.y() = acc["y"].get<double>();
            imu_data.linear_acceleration.z() = acc["z"].get<double>();
        } else {
            imu_data.linear_acceleration.setZero();
        }

        if (j.contains("angular_velocity")) {
            auto gyro = j["angular_velocity"];
            imu_data.angular_velocity.x() = gyro["x"].get<double>();
            imu_data.angular_velocity.y() = gyro["y"].get<double>();
            imu_data.angular_velocity.z() = gyro["z"].get<double>();
        } else {
            imu_data.angular_velocity.setZero();
        }

        if (j.contains("orientation")) {
            auto quat = j["orientation"];
            imu_data.orientation.w() = quat["w"].get<double>();
            imu_data.orientation.x() = quat["x"].get<double>();
            imu_data.orientation.y() = quat["y"].get<double>();
            imu_data.orientation.z() = quat["z"].get<double>();
        } else {
            imu_data.orientation.setIdentity();
        }

        return true;
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[ImuParser] JSON parse error: " << e.what() << std::endl;
        return false;
    }
}

} // namespace hdl_graph_slam_dora
