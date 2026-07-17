#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <queue>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <limits>

extern "C" {
#include "node_api.h"
}

#include "hdl_graph_slam_dora/config_manager.h"
#include "hdl_graph_slam_dora/binary_serialization.h"
#include "hdl_graph_slam_dora/imu_parser.h"
#include "hdl_graph_slam_dora/prefiltering_module.h"
#include "hdl_graph_slam_dora/scan_matching_odometry_module.h"
#include "hdl_graph_slam_dora/graph_slam_module.h"
#include "hdl_graph_slam_dora/map_saver.h"

using namespace hdl_graph_slam_dora;

std::atomic<bool> g_shutdown_requested(false);

struct PointCloudData {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double timestamp;
};

class HdlGraphSlamDoraNode {
public:
    HdlGraphSlamDoraNode(void* dora_context, const ConfigManager& config)
        : dora_context_(dora_context)
        , cloud_count_(0)
        , imu_count_(0)
        , keyframe_count_(0)
        , imu_buffer_duration_(2.0)
    {
        std::cout << "[Constructor] Initializing modules..." << std::endl;
        // 初始化模块
        std::cout << "[Constructor] Loading prefiltering config..." << std::endl;
        prefiltering_.loadConfig(config.getNode("prefiltering"));
        std::cout << "[Constructor] Loading odometry config..." << std::endl;
        odometry_.loadConfig(config.getNode("scan_matching_odometry"));
        std::cout << "[Constructor] Loading graph_slam config..." << std::endl;
        graph_slam_.loadConfig(config.getNode("graph_slam"),
                               config.getNode("loop_closure"),
                               config.getNode("constraints"));

        // 加载 IMU 时间偏移
        if (config.getNode("constraints") && config.getNode("constraints")["imu"]) {
            imu_time_offset_ = config.getNode("constraints")["imu"]["time_offset"].as<double>(0.0);
        } else {
            imu_time_offset_ = 0.0;
        }

        std::cout << "[Constructor] Loading map_saver config..." << std::endl;
        map_saver_.loadConfig(config.getNode("map_saving"));
        std::cout << "[Constructor] All modules initialized successfully" << std::endl;
        std::cout << "[Constructor] IMU time offset: " << imu_time_offset_ << "s" << std::endl;
    }

    void run() {
        std::cout << "[Main] HDL Graph SLAM Dora Node started" << std::endl;
        std::cout << "[Main] Waiting for point cloud data..." << std::endl;

        while (!g_shutdown_requested.load()) {
            void* event = dora_next_event(dora_context_);

            // dora_next_event 返回 null 可能是因为 Ctrl+C 或其他终止信号
            if (!event) {
                std::cout << "[Main] dora_next_event returned null, exiting loop" << std::endl;
                break;
            }

            DoraEventType event_type = read_dora_event_type(event);

            if (event_type == DoraEventType_Stop) {
                std::cout << "[Main] Received Stop event" << std::endl;
                free_dora_event(event);
                break;
            }

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

            free_dora_event(event);
        }

        std::cout << "[Main] Event loop exited" << std::endl;
    }

    void shutdown() {
        std::cout << "[Shutdown] Shutting down gracefully..." << std::endl;

        // 最后一次优化
        graph_slam_.optimize();

        // 保存地图
        std::cout << "[Shutdown] Saving map..." << std::endl;
        auto keyframes = graph_slam_.getKeyFrames();
        if (map_saver_.saveMap(keyframes)) {
            std::cout << "[Shutdown] Map saved successfully. Total keyframes: " << keyframes.size() << std::endl;
        } else {
            std::cerr << "[Shutdown] Failed to save map!" << std::endl;
        }

        // 打印统计信息
        std::cout << "[Shutdown] Statistics:" << std::endl;
        std::cout << "  Received clouds:  " << cloud_count_ << std::endl;
        std::cout << "  Processed clouds: " << cloud_count_ << std::endl;
        std::cout << "  Keyframes:        " << keyframe_count_ << std::endl;
        std::cout << "  IMU messages:     " << imu_count_ << std::endl;

        std::cout << "[Shutdown] Shutdown complete." << std::endl;
    }

private:
    void handlePointCloudInput(const char* data, size_t len) {
        double timestamp;
        auto cloud = parsePointCloudBinary(data, len, timestamp);
        if (!cloud) {
            std::cerr << "[Main] Failed to parse point cloud" << std::endl;
            return;
        }

        cloud_count_++;

        // 预处理
        auto filtered = prefiltering_.process(cloud);
        if (!filtered || filtered->empty()) {
            std::cerr << "[Main] Prefiltering failed or empty cloud" << std::endl;
            return;
        }

        // 扫描匹配里程计
        KeyFrame::Ptr keyframe;
        bool is_keyframe = odometry_.processFrame(filtered, timestamp, keyframe);

        if (cloud_count_ % 10 == 0) {
            auto pose = odometry_.getCurrentPose();
            Eigen::Vector3d trans = pose.translation();
            std::cout << "[Main] Processed cloud #" << cloud_count_
                      << " | Input: " << cloud->size()
                      << " | Filtered: " << filtered->size()
                      << " | Pose: [" << trans.x() << ", " << trans.y() << ", " << trans.z() << "]"
                      << " | Keyframe: " << (is_keyframe ? "YES" : "NO") << std::endl;
        }

        // 如果是关键帧，添加到图
        if (is_keyframe) {
            keyframe_count_++;

            // 关联 IMU 数据到关键帧
            associateImuToKeyframe(keyframe);

            graph_slam_.addKeyFrame(keyframe);
        }
    }

    void handleImuInput(const char* data, size_t len) {
        ImuData imu_data;
        if (!parseImuJson(data, len, imu_data)) {
            return;
        }

        // 应用时间偏移
        imu_data.timestamp += imu_time_offset_;

        imu_count_++;

        // 添加到 IMU 缓冲区
        {
            std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
            imu_buffer_.push_back(imu_data);

            // 清理旧数据（保留最近2秒）
            while (!imu_buffer_.empty() &&
                   (imu_data.timestamp - imu_buffer_.front().timestamp) > imu_buffer_duration_) {
                imu_buffer_.pop_front();
            }
        }

        if (imu_count_ % 100 == 0) {
            std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
            std::cout << "[Main] Received IMU #" << imu_count_
                      << " | Timestamp: " << std::fixed << imu_data.timestamp
                      << " | Buffer size: " << imu_buffer_.size() << std::endl;
        }
    }

    void associateImuToKeyframe(KeyFrame::Ptr& keyframe) {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);

        if (imu_buffer_.empty()) {
            return;  // 没有 IMU 数据
        }

        // 查找最接近关键帧时间戳的 IMU 数据
        double kf_timestamp = keyframe->timestamp;

        // 检查时间戳是否在 IMU 数据范围内
        if (kf_timestamp < imu_buffer_.front().timestamp ||
            kf_timestamp > imu_buffer_.back().timestamp) {
            return;  // 关键帧时间戳不在 IMU 缓冲区范围内
        }

        // 找到最接近的 IMU 数据
        const ImuData* closest_imu = nullptr;
        double min_dt = std::numeric_limits<double>::max();

        for (const auto& imu : imu_buffer_) {
            double dt = std::abs(imu.timestamp - kf_timestamp);
            if (dt < min_dt) {
                min_dt = dt;
                closest_imu = &imu;
            }

            // 如果时间差开始增大，说明已经过了最接近的点
            if (imu.timestamp > kf_timestamp && dt > min_dt) {
                break;
            }
        }

        // 检查时间差是否在阈值内（0.2秒）
        const double TIME_THRESHOLD = 0.2;
        if (closest_imu && min_dt < TIME_THRESHOLD) {
            // 关联 IMU 数据到关键帧
            keyframe->acceleration = closest_imu->linear_acceleration;
            keyframe->orientation = closest_imu->orientation;

            // 确保四元数 w 为正
            if (keyframe->orientation->w() < 0.0) {
                keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
            }

            std::cout << "[IMU Association] Keyframe #" << keyframe_count_
                      << " associated with IMU data (dt=" << std::fixed << min_dt << "s)"
                      << " | acc: [" << keyframe->acceleration->transpose() << "]"
                      << " | quat: [" << keyframe->orientation->w() << ", "
                      << keyframe->orientation->x() << ", "
                      << keyframe->orientation->y() << ", "
                      << keyframe->orientation->z() << "]" << std::endl;
        }
    }

    void* dora_context_;
    int cloud_count_;
    int imu_count_;
    int keyframe_count_;

    // IMU 相关
    double imu_time_offset_;
    double imu_buffer_duration_;
    std::mutex imu_buffer_mutex_;
    std::deque<ImuData> imu_buffer_;

    PrefilteringModule prefiltering_;
    ScanMatchingOdometryModule odometry_;
    HdlGraphSlamModule graph_slam_;
    MapSaver map_saver_;
};

HdlGraphSlamDoraNode* g_node_instance = nullptr;

void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n[Signal] Received SIGINT (Ctrl+C), setting shutdown flag..." << std::endl;
        g_shutdown_requested.store(true);
        // 不在这里调用 shutdown，让 main 函数统一处理
    }
}

int main(int argc, char** argv) {
    
    std::signal(SIGINT, signalHandler);

    // 加载配置
    std::string config_path = "../modules/mapping/hdl_graph_slam_dora/config/hdl_slam_config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    ConfigManager config;
    if (!config.loadFromFile(config_path)) {
        std::cerr << "[Main] Failed to load config from: " << config_path << std::endl;
        return 1;
    }

    // 初始化 Dora
    void* dora_context = init_dora_context_from_env();
    if (!dora_context) {
        std::cerr << "[Main] Failed to init Dora context" << std::endl;
        return 1;
    }

    // 创建节点
    HdlGraphSlamDoraNode node(dora_context, config);
    g_node_instance = &node;

    // 运行
    node.run();

    // 总是执行 shutdown 以确保地图保存
    std::cout << "[Main] Calling shutdown..." << std::endl;
    node.shutdown();

    free_dora_context(dora_context);
    return 0;
}
