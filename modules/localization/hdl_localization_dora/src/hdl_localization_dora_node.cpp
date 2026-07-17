#include <iostream>
#include <string>
#include <deque>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>
#include <chrono>
#include <limits>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pclomp/ndt_omp.h>

#include "json_serialization.h"
#include "binary_serialization.h"
#include "config_manager.h"
#include <hdl_localization/pose_estimator.hpp>

extern "C"{
#include "node_api.h"
}

using namespace hdl_localization;

/**
 * @brief Point cloud task for the NDT worker thread
 */
struct CloudTask {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double timestamp;
};

/**
 * @brief HDL Localization Dora Node
 */
class HdlLocalizationDoraNode {
public:
    HdlLocalizationDoraNode(void* dora_context)
        : dora_context_(dora_context),
          config_(ConfigManager::getInstance()),
          last_pointcloud_time_(0.0),
          pose_seq_(0),
          dropped_frame_count_(0),
          processed_count_(0),
          corrected_count_(0),
          consecutive_skips_(0),
          ever_corrected_(false),
          scan_min_range_(0.0),
          scan_max_range_(0.0),
          initialized_(false),
          stop_worker_(false) {
    }

    ~HdlLocalizationDoraNode() {
        stopWorker();
        if (dora_context_) {
            free_dora_context(dora_context_);
        }
    }

    bool initialize() {
        std::cout << "=== HDL Localization Dora Node Initialization ===" << std::endl;

        // Load global map
        if (!loadGlobalMap()) {
            std::cerr << "Failed to load global map" << std::endl;
            return false;
        }

        // Setup registration (NDT)
        if (!setupRegistration()) {
            std::cerr << "Failed to setup registration" << std::endl;
            return false;
        }

        // Initialize pose estimator
        initializePoseEstimator();

        // Setup downsampling filter
        double downsample_resolution = config_.getScanMatchingDownsampleResolution();
        voxel_filter_.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);

        // Range crop bounds (0 = disabled). Removing far/sparse and too-near points
        // lowers the NDT fitness floor and cuts per-frame cost.
        scan_min_range_ = config_.getScanMatchingMinRange();
        scan_max_range_ = config_.getScanMatchingMaxRange();
        std::cout << "Scan range crop: [" << scan_min_range_ << ", "
                  << (scan_max_range_ > 0.0 ? std::to_string(scan_max_range_) : std::string("inf"))
                  << "] m, downsample=" << downsample_resolution << "m" << std::endl;

        // Start NDT worker thread
        stop_worker_ = false;
        worker_thread_ = std::thread(&HdlLocalizationDoraNode::workerLoop, this);
        std::cout << "NDT worker thread started" << std::endl;

        std::cout << "Initialization completed successfully" << std::endl;
        std::cout << "=== IMU fusion: " << (config_.isImuEnabled() ? "ENABLED" : "DISABLED")
                  << " | cool_time=" << config_.getCoolTimeDuration() << "s"
                  << " | invert_acc=" << config_.shouldInvertAcceleration()
                  << " | invert_gyro=" << config_.shouldInvertGyroscope() << " ===" << std::endl;
        initialized_ = true;
        return true;
    }

    void run() {
        if (!initialized_) {
            std::cerr << "Node not initialized" << std::endl;
            return;
        }

        std::cout << "Starting event loop..." << std::endl;

        while (true) {
            void* event = dora_next_event(dora_context_);
            if (!event) {
                std::cerr << "Unexpected end of event" << std::endl;
                break;
            }

            DoraEventType event_type = read_dora_event_type(event);

            if (event_type == DoraEventType_Input) {
                char* data;
                size_t data_len;
                char* id;
                size_t id_len;

                read_dora_input_data(event, &data, &data_len);
                read_dora_input_id(event, &id, &id_len);

                std::string topic_id(id, id_len);

                if (topic_id == "pointcloud") {
                    handlePointCloudInput(data, data_len);
                } else if (topic_id == "imu") {
                    handleImuInput(data, data_len);
                }
            }
            else if (event_type == DoraEventType_Stop) {
                std::cout << "Received stop event" << std::endl;
                free_dora_event(event);
                break;
            }

            free_dora_event(event);
        }

        stopWorker();
        std::cout << "Event loop finished" << std::endl;
    }

private:
    bool loadGlobalMap() {
        std::string map_path = config_.getMapPath();
        std::cout << "Loading global map from: " << map_path << std::endl;

        globalmap_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *globalmap_) == -1) {
            std::cerr << "Failed to load map from: " << map_path << std::endl;
            return false;
        }

        std::cout << "Loaded map with " << globalmap_->points.size() << " points" << std::endl;

        // Downsample map
        double map_downsample_resolution = config_.getMapDownsampleResolution();
        if (map_downsample_resolution > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setLeafSize(map_downsample_resolution, map_downsample_resolution, map_downsample_resolution);
            voxel_filter.setInputCloud(globalmap_);

            auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            voxel_filter.filter(*filtered);

            std::cout << "Downsampled map: " << globalmap_->points.size()
                      << " -> " << filtered->points.size() << " points" << std::endl;
            globalmap_ = filtered;
        }

        return true;
    }

    bool setupRegistration() {
        std::string ndt_method = config_.getNdtMethod();
        std::cout << "Setting up registration method: " << ndt_method << std::endl;

        if (ndt_method == "NDT_OMP") {
            auto ndt = std::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();

            double resolution = config_.getNdtResolution();
            ndt->setResolution(resolution);
            ndt->setTransformationEpsilon(config_.getNdtTransformationEpsilon());
            ndt->setMaximumIterations(config_.getNdtMaxIterations());
            ndt->setNumThreads(config_.getNdtNumThreads());

            std::string neighbor_search_method = config_.getNdtNeighborSearchMethod();
            if (neighbor_search_method == "DIRECT7") {
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            } else if (neighbor_search_method == "DIRECT1") {
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
            }

            ndt->setInputTarget(globalmap_);
            registration_ = ndt;

            std::cout << "NDT_OMP configured successfully:" << std::endl;
            std::cout << "  Resolution: " << resolution << std::endl;
            std::cout << "  Max iterations: " << config_.getNdtMaxIterations() << std::endl;
            std::cout << "  Transformation epsilon: " << config_.getNdtTransformationEpsilon() << std::endl;
            std::cout << "  Threads: " << config_.getNdtNumThreads() << std::endl;
            return true;
        } else {
            std::cerr << "Unsupported registration method: " << ndt_method << std::endl;
            return false;
        }
    }

    void initializePoseEstimator() {
        Eigen::Vector3f initial_pos = config_.getInitialPosition();
        Eigen::Quaternionf initial_quat = config_.getInitialOrientation();
        double cool_time = config_.getCoolTimeDuration();

        std::cout << "Initializing pose estimator at position: ["
                  << initial_pos.x() << ", " << initial_pos.y() << ", " << initial_pos.z() << "]" << std::endl;

        pose_estimator_.reset(new PoseEstimator(registration_, initial_pos, initial_quat, cool_time));
    }

    void handlePointCloudInput(const char* data, size_t len) {
        double timestamp = 0.0;
        auto cloud = binaryToPointCloud(data, len, timestamp);

        if (!cloud || cloud->empty()) {
            std::cerr << "Binary point cloud parse failed or cloud is empty" << std::endl;
            return;
        }

        // Validate timestamp
        if (timestamp <= 0.0 || !std::isfinite(timestamp)) {
            std::cerr << "Invalid timestamp: " << timestamp << std::endl;
            return;
        }

        // Check for at least one finite point
        bool has_valid_points = false;
        for (const auto& pt : cloud->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                has_valid_points = true;
                break;
            }
        }

        if (!has_valid_points) {
            std::cerr << "Point cloud contains no valid points (all NaN/Inf)" << std::endl;
            return;
        }

        // Enqueue for async NDT processing.
        // Keep only the latest frame pending: when the worker is still busy, older
        // frames are stale for localization, so drop them to minimize latency.
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            while (cloud_queue_.size() >= kMaxQueueSize) {
                cloud_queue_.pop();
                if (++dropped_frame_count_ % 50 == 0) {
                    std::cerr << "[WARN] NDT worker behind: dropped " << dropped_frame_count_
                              << " frames total (NDT slower than input rate)" << std::endl;
                }
            }
            cloud_queue_.push({cloud, timestamp});
        }
        queue_cv_.notify_one();
    }

    /**
     * @brief NDT worker thread: dequeues cloud tasks and runs scan matching
     */
    void workerLoop() {
        while (true) {
            CloudTask task;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] {
                    return stop_worker_ || !cloud_queue_.empty();
                });
                if (stop_worker_ && cloud_queue_.empty()) {
                    break;
                }
                task = std::move(cloud_queue_.front());
                cloud_queue_.pop();
            }
            processPointCloud(task.cloud, task.timestamp);
        }
        std::cout << "NDT worker thread exited" << std::endl;
    }

    /**
     * @brief Signal the worker thread to stop and wait for it to join
     */
    void stopWorker() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_worker_ = true;
        }
        queue_cv_.notify_one();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    void handleImuInput(const char* data, size_t len) {
        try {
            std::string json_str(data, len);
            nlohmann::json json_data = nlohmann::json::parse(json_str);

            ImuData imu;
            if (!JsonSerializer::jsonToImu(json_data, imu.timestamp, imu.acc, imu.gyro)) {
                return;
            }

            if (imu.timestamp <= 0.0 || !std::isfinite(imu.timestamp)) {
                std::cerr << "Invalid IMU timestamp: " << imu.timestamp << std::endl;
                return;
            }

            if (!imu.acc.allFinite()) {
                std::cerr << "Invalid IMU acceleration (contains NaN/Inf)" << std::endl;
                return;
            }

            if (!imu.gyro.allFinite()) {
                std::cerr << "Invalid IMU gyroscope (contains NaN/Inf)" << std::endl;
                return;
            }

            if (config_.shouldInvertAcceleration()) {
                imu.acc = -imu.acc;
            }
            if (config_.shouldInvertGyroscope()) {
                imu.gyro = -imu.gyro;
            }

            float acc_norm = imu.acc.norm();
            if (acc_norm < 5.0 || acc_norm > 15.0) {
                static int warning_count = 0;
                if (++warning_count % 100 == 0) {
                    std::cerr << "[WARNING] Abnormal IMU acceleration magnitude: "
                              << acc_norm << " m/s² (expected ~9.8)" << std::endl;
                }
            }

            // MID360: 点云与 IMU 均使用同一硬件时钟，无需时钟偏差补偿
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);

                imu_buffer_.push_back(imu);
                if (imu_buffer_.size() > 1000) {
                    imu_buffer_.pop_front();
                }
            }

        } catch (const std::exception& e) {
            std::cerr << "Error processing IMU: " << e.what() << std::endl;
        }
    }

    void processPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double timestamp) {
        using Clock = std::chrono::steady_clock;
        using Ms = std::chrono::duration<double, std::milli>;

        // Range crop: drop points too far (sparse in map → inflate fitness) or too near
        pcl::PointCloud<pcl::PointXYZI>::Ptr cropped;
        if (scan_max_range_ > 0.0 || scan_min_range_ > 0.0) {
            cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            cropped->points.reserve(cloud->points.size());
            const float min_sq = static_cast<float>(scan_min_range_ * scan_min_range_);
            const float max_sq = scan_max_range_ > 0.0
                                     ? static_cast<float>(scan_max_range_ * scan_max_range_)
                                     : std::numeric_limits<float>::max();
            for (const auto& pt : cloud->points) {
                float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
                if (r2 >= min_sq && r2 <= max_sq) {
                    cropped->points.push_back(pt);
                }
            }
            cropped->width = static_cast<uint32_t>(cropped->points.size());
            cropped->height = 1;
            cropped->is_dense = false;
        } else {
            cropped = cloud;
        }

        // Downsample
        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        voxel_filter_.setInputCloud(cropped);
        voxel_filter_.filter(*filtered);

        if (filtered->empty()) {
            std::cerr << "Filtered cloud is empty (raw=" << cloud->size()
                      << ", cropped=" << cropped->size() << ")" << std::endl;
            return;
        }

        // Predict with IMU data
        if (config_.isImuEnabled()) {
            std::lock_guard<std::mutex> lock(imu_mutex_);

            if (!imu_buffer_.empty()) {
                double time_diff = timestamp - imu_buffer_.back().timestamp;
                if (std::abs(time_diff) > 0.5) {
                    std::cerr << "[WARNING] Large time difference between IMU and PointCloud: "
                              << time_diff << "s" << std::endl;
                }
            }

            int imu_count = 0;
            while (!imu_buffer_.empty() && imu_buffer_.front().timestamp <= timestamp) {
                const auto& imu = imu_buffer_.front();
                if (imu.timestamp > last_pointcloud_time_) {
                    pose_estimator_->predict(imu.timestamp, imu.acc, imu.gyro);
                    imu_count++;
                }
                imu_buffer_.pop_front();
            }

            if (imu_count == 0 && last_pointcloud_time_ > 0) {
                std::cerr << "[WARN] No IMU data between point clouds! "
                          << "Check IMU topic connection and timestamp sync." << std::endl;
            }
        }

        // Predict to current timestamp
        pose_estimator_->predict(timestamp);

        // Correction with scan matching (timed)
        const auto t_ndt0 = Clock::now();
        auto aligned = pose_estimator_->correct(timestamp, filtered);
        const double ndt_ms = Ms(Clock::now() - t_ndt0).count();

        if (!aligned || aligned->empty()) {
            std::cerr << "Scan matching failed or returned empty cloud" << std::endl;
            return;
        }

        // Get estimated pose
        Eigen::Matrix4f pose = pose_estimator_->matrix();

        // --- Localization status logging ---
        logLocalizationStatus(timestamp, ndt_ms, filtered->size(), cloud->size());

        // Check for NaN
        if (!pose.allFinite()) {
            std::cerr << "ERROR: Pose contains NaN or Inf values!" << std::endl;
            std::cerr << "Pose matrix:\n" << pose << std::endl;
            return;
        }

        // Publish results
        publishPose(pose, timestamp);

        if (config_.shouldPublishAlignedPoints() && aligned) {
            publishAlignedPoints(aligned, timestamp);
        }

        last_pointcloud_time_ = timestamp;
    }

    static const char* statusStr(PoseEstimator::CorrectionStatus s) {
        switch (s) {
            case PoseEstimator::CorrectionStatus::Corrected:          return "CORRECTED";
            case PoseEstimator::CorrectionStatus::SkippedHighFitness: return "SKIP_FITNESS";
            case PoseEstimator::CorrectionStatus::NotConverged:       return "NOT_CONVERGED";
            default:                                                  return "NONE";
        }
    }

    /**
     * @brief Emit human-readable localization status: pose, fitness, accept/reject,
     *        point counts, NDT timing, and lock-loss transitions.
     */
    void logLocalizationStatus(double timestamp, double ndt_ms, size_t src_pts, size_t raw_pts) {
        const auto status = pose_estimator_->last_status();
        const double fitness = pose_estimator_->last_fitness();
        const Eigen::Vector3f p = pose_estimator_->pos();

        ++processed_count_;
        const bool corrected = (status == PoseEstimator::CorrectionStatus::Corrected);
        if (corrected) {
            ++corrected_count_;
            if (!ever_corrected_) {
                ever_corrected_ = true;
                std::cout << "[HDL] first successful correction @#" << processed_count_
                          << " fitness=" << fitness
                          << " pos=(" << p.x() << ", " << p.y() << ")" << std::endl;
            }
            consecutive_skips_ = 0;
        } else {
            ++consecutive_skips_;
            // Flag the moment localization starts losing lock (once per lost streak)
            if (consecutive_skips_ == kLockLossStreak && ever_corrected_) {
                std::cerr << "[WARN] localization losing lock @#" << processed_count_
                          << " after " << corrected_count_ << " good frames"
                          << " | status=" << statusStr(status)
                          << " fitness=" << fitness
                          << " pos=(" << p.x() << ", " << p.y() << ")" << std::endl;
            }
        }

        // Periodic heartbeat so the log always shows what the estimator is doing
        if (processed_count_ % kStatusLogEvery == 0) {
            std::cout << "[HDL] #" << processed_count_
                      << " pos=(" << p.x() << ", " << p.y() << ", " << p.z() << ")"
                      << " fitness=" << fitness
                      << " " << statusStr(status)
                      << " src=" << src_pts << "/" << raw_pts
                      << " ndt=" << ndt_ms << "ms"
                      << " ok=" << corrected_count_ << "/" << processed_count_
                      << " dropped=" << dropped_frame_count_ << std::endl;
        }
    }

    void publishPose(const Eigen::Matrix4f& pose, double timestamp) {
        // 将雷达坐标系的pose转换到base_link坐标系
        // base_link在地图坐标系下的位姿 = 雷达在地图坐标系下的位姿 * 雷达到base_link的变换
        Eigen::Matrix4f lidar_to_base = Eigen::Matrix4f::Identity();

        // 获取平移量
        Eigen::Vector3f translation = config_.getLidarToBaseLinkTranslation();
        lidar_to_base(0, 3) = translation.x();
        lidar_to_base(1, 3) = translation.y();
        lidar_to_base(2, 3) = translation.z();

        // 获取旋转量（欧拉角）并转换为旋转矩阵
        Eigen::Vector3f rotation = config_.getLidarToBaseLinkRotation();
        Eigen::AngleAxisf rollAngle(rotation.x(), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(rotation.y(), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(rotation.z(), Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotation_matrix = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
        lidar_to_base.block<3, 3>(0, 0) = rotation_matrix;

        // 计算base_link在地图坐标系下的位姿
        Eigen::Matrix4f base_link_pose = pose * lidar_to_base;

        nlohmann::json json_data = JsonSerializer::poseToJson(base_link_pose, timestamp, pose_seq_++);
        std::string json_str = json_data.dump();

        std::string topic = config_.getPoseTopic();
        int result = dora_send_output(dora_context_,
                                      const_cast<char*>(topic.c_str()), topic.length(),
                                      const_cast<char*>(json_str.c_str()), json_str.length());

        if (result != 0) {
            std::cerr << "Failed to publish pose" << std::endl;
        }
    }

    void publishAlignedPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double timestamp) {
        nlohmann::json json_data = JsonSerializer::pointCloudToJson(cloud, timestamp);
        std::string json_str = json_data.dump();

        std::string topic = config_.getAlignedPointsTopic();
        int result = dora_send_output(dora_context_,
                                      const_cast<char*>(topic.c_str()), topic.length(),
                                      const_cast<char*>(json_str.c_str()), json_str.length());

        if (result != 0) {
            std::cerr << "Failed to publish aligned points" << std::endl;
        }
    }

private:
    // Dora context
    void* dora_context_;

    // Configuration
    ConfigManager& config_;

    // Core components
    std::unique_ptr<PoseEstimator> pose_estimator_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap_;

    // Registration (NDT)
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration_;

    // IMU data buffer
    std::deque<ImuData> imu_buffer_;
    std::mutex imu_mutex_;

    // NDT worker thread
    std::thread worker_thread_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<CloudTask> cloud_queue_;
    bool stop_worker_;
    static constexpr size_t kMaxQueueSize = 1;

    // State
    double last_pointcloud_time_;
    uint32_t pose_seq_;

    // Count of point cloud frames dropped because the NDT worker fell behind
    uint64_t dropped_frame_count_;

    // Localization status tracking (for logging)
    uint64_t processed_count_;
    uint64_t corrected_count_;
    int consecutive_skips_;
    bool ever_corrected_;
    static constexpr uint64_t kStatusLogEvery = 10;
    static constexpr int kLockLossStreak = 5;

    // Scan range crop bounds in meters (0 = disabled)
    double scan_min_range_;
    double scan_max_range_;

    bool initialized_;

    // Downsampling filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
};

int main(int argc, char** argv) {
    std::cout << "=== HDL Localization Dora Node ===" << std::endl;

    // Initialize Dora context
    void* dora_context = init_dora_context_from_env();
    if (!dora_context) {
        std::cerr << "Failed to initialize Dora context" << std::endl;
        return -1;
    }

    std::cout << "Dora context initialized" << std::endl;

    // Load configuration
    ConfigManager& config = ConfigManager::getInstance();
    std::string config_path = "../modules/localization/hdl_localization_dora/config/hdl_localization_config.json";

    // Check if config path is provided as argument
    if (argc > 1) {
        config_path = argv[1];
    }

    if (!config.loadFromFile(config_path)) {
        std::cerr << "Failed to load configuration from: " << config_path << std::endl;
        free_dora_context(dora_context);
        return -1;
    }

    // Create and initialize node
    HdlLocalizationDoraNode node(dora_context);
    if (!node.initialize()) {
        std::cerr << "Failed to initialize node" << std::endl;
        return -1;
    }

    // Run event loop
    node.run();

    std::cout << "HDL Localization Dora Node finished successfully" << std::endl;
    return 0;
}
