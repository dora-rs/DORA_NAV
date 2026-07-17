#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace hdl_localization {

/**
 * @brief Configuration manager for hdl_localization_dora
 * Singleton pattern for global configuration access
 */
class ConfigManager {
public:
    /**
     * @brief Get singleton instance
     */
    static ConfigManager& getInstance() {
        static ConfigManager instance;
        return instance;
    }

    // Delete copy constructor and assignment operator
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    /**
     * @brief Load configuration from JSON file
     * @param config_path Path to configuration file
     * @return true if successful, false otherwise
     */
    bool loadFromFile(const std::string& config_path);

    // Map configuration
    std::string getMapPath() const;
    double getMapDownsampleResolution() const;

    // Initial pose configuration
    Eigen::Vector3f getInitialPosition() const;
    Eigen::Quaternionf getInitialOrientation() const;

    // IMU configuration
    bool isImuEnabled() const;
    bool shouldInvertAcceleration() const;
    bool shouldInvertGyroscope() const;
    double getCoolTimeDuration() const;

    // NDT configuration
    std::string getNdtMethod() const;
    double getNdtResolution() const;
    std::string getNdtNeighborSearchMethod() const;
    double getNdtNeighborSearchRadius() const;
    double getNdtTransformationEpsilon() const;
    int getNdtMaxIterations() const;
    int getNdtNumThreads() const;
    double getNdtMaxFitnessScore() const;

    // Scan matching configuration
    double getScanMatchingDownsampleResolution() const;
    double getScanMatchingMinRange() const;
    double getScanMatchingMaxRange() const;

    // UKF configuration
    double getUkfProcessNoisePosition() const;
    double getUkfProcessNoiseVelocity() const;
    double getUkfProcessNoiseOrientation() const;
    double getUkfProcessNoiseAccBias() const;
    double getUkfProcessNoiseGyroBias() const;
    double getUkfMeasurementNoisePosition() const;
    double getUkfMeasurementNoiseOrientation() const;

    // Output configuration
    bool shouldPublishAlignedPoints() const;
    std::string getPoseTopic() const;
    std::string getAlignedPointsTopic() const;

    // Transform configuration
    Eigen::Vector3f getLidarToBaseLinkTranslation() const;
    Eigen::Vector3f getLidarToBaseLinkRotation() const;

private:
    ConfigManager() = default;
    nlohmann::json config_;

    // Helper function to safely get value with default
    template<typename T>
    T getValue(const std::string& path, const T& default_value) const;
};

} // namespace hdl_localization

#endif // CONFIG_MANAGER_H
