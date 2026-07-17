#include "config_manager.h"
#include <fstream>
#include <iostream>

namespace hdl_localization {

bool ConfigManager::loadFromFile(const std::string& config_path) {
    try {
        std::ifstream file(config_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open config file: " << config_path << std::endl;
            return false;
        }

        file >> config_;
        std::cout << "Configuration loaded successfully from: " << config_path << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        return false;
    }
}

// Map configuration
std::string ConfigManager::getMapPath() const {
    return config_.value("map", nlohmann::json::object()).value("pcd_file", "../maps/pcd/map.pcd");
}

double ConfigManager::getMapDownsampleResolution() const {
    return config_.value("map", nlohmann::json::object()).value("downsample_resolution", 0.1);
}

// Initial pose configuration
Eigen::Vector3f ConfigManager::getInitialPosition() const {
    auto initial_pose = config_.value("initial_pose", nlohmann::json::object());
    auto position = initial_pose.value("position", nlohmann::json::object());

    return Eigen::Vector3f(
        position.value("x", 0.0f),
        position.value("y", 0.0f),
        position.value("z", 0.0f)
    );
}

Eigen::Quaternionf ConfigManager::getInitialOrientation() const {
    auto initial_pose = config_.value("initial_pose", nlohmann::json::object());
    auto orientation = initial_pose.value("orientation", nlohmann::json::object());

    return Eigen::Quaternionf(
        orientation.value("w", 1.0f),
        orientation.value("x", 0.0f),
        orientation.value("y", 0.0f),
        orientation.value("z", 0.0f)
    );
}

// IMU configuration
bool ConfigManager::isImuEnabled() const {
    return config_.value("imu", nlohmann::json::object()).value("enabled", true);
}

bool ConfigManager::shouldInvertAcceleration() const {
    return config_.value("imu", nlohmann::json::object()).value("invert_acceleration", false);
}

bool ConfigManager::shouldInvertGyroscope() const {
    return config_.value("imu", nlohmann::json::object()).value("invert_gyroscope", false);
}

double ConfigManager::getCoolTimeDuration() const {
    return config_.value("imu", nlohmann::json::object()).value("cool_time_duration", 2.0);
}

// NDT configuration
std::string ConfigManager::getNdtMethod() const {
    return config_.value("ndt", nlohmann::json::object()).value("method", "NDT_OMP");
}

double ConfigManager::getNdtResolution() const {
    return config_.value("ndt", nlohmann::json::object()).value("resolution", 1.0);
}

std::string ConfigManager::getNdtNeighborSearchMethod() const {
    return config_.value("ndt", nlohmann::json::object()).value("neighbor_search_method", "DIRECT7");
}

double ConfigManager::getNdtNeighborSearchRadius() const {
    return config_.value("ndt", nlohmann::json::object()).value("neighbor_search_radius", 2.0);
}

double ConfigManager::getNdtTransformationEpsilon() const {
    return config_.value("ndt", nlohmann::json::object()).value("transformation_epsilon", 0.01);
}

int ConfigManager::getNdtMaxIterations() const {
    return config_.value("ndt", nlohmann::json::object()).value("max_iterations", 30);
}

int ConfigManager::getNdtNumThreads() const {
    return config_.value("ndt", nlohmann::json::object()).value("num_threads", 4);
}

double ConfigManager::getNdtMaxFitnessScore() const {
    return config_.value("ndt", nlohmann::json::object()).value("max_fitness_score", 2.0);
}

// Scan matching configuration
double ConfigManager::getScanMatchingDownsampleResolution() const {
    return config_.value("scan_matching", nlohmann::json::object()).value("downsample_resolution", 0.1);
}

double ConfigManager::getScanMatchingMinRange() const {
    return config_.value("scan_matching", nlohmann::json::object()).value("min_range", 0.0);
}

double ConfigManager::getScanMatchingMaxRange() const {
    return config_.value("scan_matching", nlohmann::json::object()).value("max_range", 0.0);
}

// UKF configuration
double ConfigManager::getUkfProcessNoisePosition() const {
    auto ukf = config_.value("ukf", nlohmann::json::object());
    auto process_noise = ukf.value("process_noise", nlohmann::json::object());
    return process_noise.value("position", 1.0);
}

double ConfigManager::getUkfProcessNoiseVelocity() const {
    auto ukf = config_.value("ukf", nlohmann::json::object());
    auto process_noise = ukf.value("process_noise", nlohmann::json::object());
    return process_noise.value("velocity", 1.0);
}

double ConfigManager::getUkfProcessNoiseOrientation() const {
    auto ukf = config_.value("ukf", nlohmann::json::object());
    auto process_noise = ukf.value("process_noise", nlohmann::json::object());
    return process_noise.value("orientation", 0.5);
}

double ConfigManager::getUkfProcessNoiseAccBias() const {
    auto ukf = config_.value("ukf", nlohmann::json::object());
    auto process_noise = ukf.value("process_noise", nlohmann::json::object());
    return process_noise.value("acc_bias", 1e-6);
}

double ConfigManager::getUkfProcessNoiseGyroBias() const {
    auto ukf = config_.value("ukf", nlohmann::json::object());
    auto process_noise = ukf.value("process_noise", nlohmann::json::object());
    return process_noise.value("gyro_bias", 1e-6);
}

double ConfigManager::getUkfMeasurementNoisePosition() const {
    auto ukf = config_.value("ukf", nlohmann::json::object());
    auto measurement_noise = ukf.value("measurement_noise", nlohmann::json::object());
    return measurement_noise.value("position", 0.01);
}

double ConfigManager::getUkfMeasurementNoiseOrientation() const {
    auto ukf = config_.value("ukf", nlohmann::json::object());
    auto measurement_noise = ukf.value("measurement_noise", nlohmann::json::object());
    return measurement_noise.value("orientation", 0.001);
}

// Output configuration
bool ConfigManager::shouldPublishAlignedPoints() const {
    return config_.value("output", nlohmann::json::object()).value("publish_aligned_points", false);
}

std::string ConfigManager::getPoseTopic() const {
    return config_.value("output", nlohmann::json::object()).value("pose_topic", "pose");
}

std::string ConfigManager::getAlignedPointsTopic() const {
    return config_.value("output", nlohmann::json::object()).value("aligned_points_topic", "aligned_points");
}

// Transform configuration
Eigen::Vector3f ConfigManager::getLidarToBaseLinkTranslation() const {
    auto transform = config_.value("transform", nlohmann::json::object());
    auto lidar_to_base = transform.value("lidar_to_base_link", nlohmann::json::object());

    return Eigen::Vector3f(
        lidar_to_base.value("x", 0.0f),
        lidar_to_base.value("y", 0.0f),
        lidar_to_base.value("z", 0.0f)
    );
}

Eigen::Vector3f ConfigManager::getLidarToBaseLinkRotation() const {
    auto transform = config_.value("transform", nlohmann::json::object());
    auto lidar_to_base = transform.value("lidar_to_base_link", nlohmann::json::object());

    return Eigen::Vector3f(
        lidar_to_base.value("roll", 0.0f),
        lidar_to_base.value("pitch", 0.0f),
        lidar_to_base.value("yaw", 0.0f)
    );
}

} // namespace hdl_localization
