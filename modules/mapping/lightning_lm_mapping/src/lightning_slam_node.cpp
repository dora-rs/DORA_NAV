extern "C" {
#include "node_api.h"
}

#include <yaml-cpp/yaml.h>
#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/point_def.h"
#include "common/pose_rpy.h"
#include "core/lightning_math.hpp"
#include "core/lio/laser_mapping.h"
#include "core/system/slam.h"
#include "utils/timer.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace lightning;

std::atomic<bool> to_exit_process{false};

// 6DOF 位姿二进制格式（与 NDT mapping 保持一致）
struct PoseOutput {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

/**
 * 模仿 PointCloudPreprocess 的 PCL -> PCL 预处理器
 */
class PointCloudPreprocessPCL {
   public:
    PointCloudPreprocessPCL() = default;

    /// 模仿原 PointCloudPreprocess 的参数名称
    double& Blind() { return blind_; }
    int& PointFilterNum() { return point_filter_num_; }
    void SetHeightROI(float height_max, float height_min) {
        height_max_ = height_max;
        height_min_ = height_min;
    }
    // 与 PointCloudPreprocess 保持一致的参数名
    int point_filter_num_ = 1;
    double blind_ = 0.01;
    float height_max_ = 1.0;
    float height_min_ = -1.0;
};

class DoraSlamNode {
   public:
    DoraSlamNode(const std::string& yaml_path, void* dora_context)
        : yaml_path_(yaml_path), dora_context_(dora_context) {
        // 初始化 SLAM 系统
        SlamSystem::Options options;
        options.online_mode_ = true;
        slam_ = std::make_shared<SlamSystem>(options);
        if (!slam_->Init(yaml_path_)) {
            LOG(ERROR) << "failed to init slam";
            return;
        }

        // 初始化 PCL 预处理器
        preprocess_pcl_ = std::make_shared<PointCloudPreprocessPCL>();
        if (!InitPreprocess()) {
            LOG(ERROR) << "failed to init pointcloud preprocess";
            return;
        }

        slam_->StartSLAM("new_map");
        LOG(INFO) << "DORA SLAM Node (PCL Preprocess) initialized and started.";
    }

    void Run(void* dora_context) {
        LOG(INFO) << "DORA event loop started.";

        while (!to_exit_process) {
            void* event = dora_next_event(dora_context);
            if (event == NULL) {
                printf("[lightning-lm] ERROR: unexpected end of event\n");
                continue;
            }

            enum DoraEventType ty = read_dora_event_type(event);

            if (ty == DoraEventType_Input) {
                char* id_ptr = nullptr;
                size_t id_len = 0;
                read_dora_input_id(event, &id_ptr, &id_len);
                std::string input_id(id_ptr, id_len);

                if (id_ptr == nullptr || id_len == 0) {
                    continue;
                }

                if (input_id == "imu") {
                    HandleImu(event);
                } else if (input_id == "pointcloud") {
                    HandleLidar(event);
                }
            } else if (ty == DoraEventType_Stop) {
                printf("[lightning-lm] received stop event\n");
                to_exit_process = true;
            } else if (ty == DoraEventType_Error) {
                printf("[lightning-lm] received error event (e.g., Ctrl+C)\n");
                to_exit_process = true;
            } else if (ty == DoraEventType_InputClosed) {
                printf("[lightning-lm] received input closed event\n");
                to_exit_process = true;
            } else {
                printf("[lightning-lm] received unexpected event: %d\n", ty);
            }

            free_dora_event(event);
        }

        printf("[lightning-lm] event loop exited normally\n");
    }

    void SaveMap() {
        std::string save_path = "../modules/mapping/maps";

        try {
            auto yaml = YAML::LoadFile(yaml_path_);
            if (yaml["system"] && yaml["system"]["map_path"]) {
                save_path = yaml["system"]["map_path"].as<std::string>();
            }
        } catch (...) {
            LOG(WARNING) << "Failed to read map_path from config, using default: " << save_path;
        }

        LOG(INFO) << "Saving map to: " << save_path;
        slam_->SaveMap(save_path);
        LOG(INFO) << "Map saved successfully to: " << save_path;
    }

   private:
    bool InitPreprocess() {
        try {
            auto yaml = YAML::LoadFile(yaml_path_);
            preprocess_pcl_->Blind() = yaml["fasterlio"]["blind"].as<double>();
            preprocess_pcl_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();

            float height_max = yaml["roi"]["height_max"].as<float>();
            float height_min = yaml["roi"]["height_min"].as<float>();
            preprocess_pcl_->SetHeightROI(height_max, height_min);
        } catch (...) {
            LOG(ERROR) << "Exception during preprocess init from YAML.";
            return false;
        }
        return true;
    }

    void HandleImu(void* input_event) {
        char* data_ptr = nullptr;
        size_t data_len = 0;
        read_dora_input_data(input_event, &data_ptr, &data_len);

        if (data_ptr == nullptr || data_len == 0) return;

        // 使用 string_view 避免内存拷贝（C++17）
        std::string_view json_view(data_ptr, data_len);

        try {
            auto data = json::parse(json_view);

            IMUPtr imu = std::make_shared<IMU>();

            // 解析时间戳（Livox 格式：header.timestamp 为秒的浮点数）
            imu->timestamp = data["header"]["timestamp"].get<double>();

            // 解析线加速度
            // 注意：Livox 驱动输出的单位是 g（重力加速度），需要转换为 m/s²
            constexpr double G = 9.80665;  // 标准重力加速度 m/s²
            imu->linear_acceleration = Vec3d(
                data["linear_acceleration"]["x"].get<double>() * G,
                data["linear_acceleration"]["y"].get<double>() * G,
                data["linear_acceleration"]["z"].get<double>() * G
            );

            // 解析角速度 (rad/s)
            imu->angular_velocity = Vec3d(
                data["angular_velocity"]["x"].get<double>(),
                data["angular_velocity"]["y"].get<double>(),
                data["angular_velocity"]["z"].get<double>()
            );

            // 送入 SLAM 系统处理
            if (slam_) {
                slam_->ProcessIMU(imu);
            }
        } catch (const json::exception& e) {
            std::cerr << "[lightning-lm] IMU JSON parse error: " << e.what() << std::endl;
        }
    }

    // ---------------------------------------------------------------------------
    // ParseLivoxPointCloud
    // 解析 Livox 驱动输出的二进制点云格式
    // 格式: [0..7] double timestamp | [8..11] uint32 point_count | [12..] point_count×16 bytes (x,y,z,intensity)
    // ---------------------------------------------------------------------------
    bool ParseLivoxPointCloud(const uint8_t* data, size_t len, double& timestamp, pcl::PointCloud<PointType>::Ptr& cloud) {
        constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t);  // 12 bytes
        constexpr size_t POINT_SIZE = 4 * sizeof(float);                   // 16 bytes

        // 1. 验证最小头部长度
        if (len < HEADER_SIZE) {
            std::cerr << "[lightning-lm] ParseLivoxPointCloud: buffer too small (" << len << " bytes)\n";
            return false;
        }

        // 2. 解析头部：时间戳 + 点数量
        std::memcpy(&timestamp, data, sizeof(double));
        uint32_t point_count = 0;
        std::memcpy(&point_count, data + sizeof(double), sizeof(uint32_t));

        // 3. 验证数据完整性
        const size_t expected_len = HEADER_SIZE + point_count * POINT_SIZE;
        if (len != expected_len) {
            std::cerr << "[lightning-lm] ParseLivoxPointCloud: data length mismatch. "
                      << "Expected: " << expected_len << ", Got: " << len << std::endl;
            return false;
        }

        // 4. 预分配点云空间
        cloud->clear();
        cloud->reserve(point_count);

        // 5. 解析点云数据，同时应用预处理过滤
        size_t count = 0;
        const uint8_t* point_data = data + HEADER_SIZE;

        for (uint32_t i = 0; i < point_count; ++i) {
            count++;

            // 采样过滤：每 point_filter_num_ 个点取一个
            if (count % preprocess_pcl_->point_filter_num_ != 0) continue;

            // 解析点坐标和强度
            float x, y, z, intensity;
            const size_t offset = i * POINT_SIZE;
            std::memcpy(&x, point_data + offset + 0, sizeof(float));
            std::memcpy(&y, point_data + offset + 4, sizeof(float));
            std::memcpy(&z, point_data + offset + 8, sizeof(float));
            std::memcpy(&intensity, point_data + offset + 12, sizeof(float));

            // 跳过无效点（NaN/Inf）
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            // 盲区过滤：距离太近的点
            double range_sq = x * x + y * y + z * z;
            if (range_sq < preprocess_pcl_->blind_ * preprocess_pcl_->blind_) continue;

            // 高度 ROI 过滤
            if (z > preprocess_pcl_->height_max_ || z < preprocess_pcl_->height_min_) continue;

            // 创建点并添加到点云
            PointType pt;
            pt.x = x;
            pt.y = y;
            pt.z = z;
            pt.intensity = intensity;

            // 时间偏移：将扫描时间均匀分布在 0~100ms 内（Livox 特性）
            pt.time = (double)i / point_count * 100.0;

            cloud->points.push_back(pt);
        }

        // 6. 设置点云元数据
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e9);  // 转换为纳秒

        return true;
    }
    void HandleLidar(void* input_event) {
        char* data_ptr = nullptr;
        size_t data_len = 0;
        read_dora_input_data(input_event, &data_ptr, &data_len);

        if (data_ptr == nullptr || data_len == 0) {
            std::cerr << "[lightning-lm] HandleLidar: received empty data" << std::endl;
            return;
        }

        const uint8_t* data = reinterpret_cast<const uint8_t*>(data_ptr);

        double timestamp = 0.0;
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

        if (ParseLivoxPointCloud(data, data_len, timestamp, cloud)) {
            // 送入 SLAM 系统处理
            slam_->ProcessLidar(cloud);

            // 发送位姿输出
            SendPose();

            // 定期发送完整地图（每50帧发送一次，用于全局地图更新）
            lidar_frame_count_++;
            if (lidar_frame_count_ % 50 == 0) {
                SendFullMap();
            }
        } else {
            std::cerr << "[lightning-lm] Failed to parse point cloud data" << std::endl;
        }
    }

    // ---------------------------------------------------------------------------
    // SendPose - 发送当前位姿（二进制格式，与 NDT mapping 一致）
    // ---------------------------------------------------------------------------
    void SendPose() {
        if (!dora_context_ || !slam_) return;

        // 获取 LIO 前端
        auto lio = slam_->GetLIO();
        if (!lio) return;

        // 获取当前状态
        auto state = lio->GetState();
        if (!state.pose_is_ok_) return;  // 检查位姿是否有效

        // 提取位置
        Vec3d translation = state.pos_;

        // 将旋转转换为 SE3，然后提取 RPY
        SE3 pose(state.rot_, translation);
        auto pose_rpy = math::SE3ToRollPitchYaw(pose);

        // 填充输出结构体
        PoseOutput pose_out;
        pose_out.x = pose_rpy.x;
        pose_out.y = pose_rpy.y;
        pose_out.z = pose_rpy.z;
        pose_out.roll = pose_rpy.roll;
        pose_out.pitch = pose_rpy.pitch;
        pose_out.yaw = pose_rpy.yaw;

        // 发送到 DORA
        std::string output_id = "pose";
        dora_send_output(dora_context_,
                        const_cast<char*>(output_id.c_str()), output_id.length(),
                        reinterpret_cast<char*>(&pose_out),
                        sizeof(PoseOutput));
    }

    // 发送完整地图（回环后或定期）
    void SendFullMap() {
        if (!dora_context_ || !slam_) return;

        auto lio = slam_->GetLIO();
        if (!lio) return;

        // 使用 LaserMapping 提供的接口获取全局地图
        // use_lio_pose=true: 使用 LIO 前端位姿
        // use_voxel=true: 使用体素降采样
        // res=0.1: 降采样分辨率 0.1m
        auto global_map = lio->GetGlobalMap(true, true, 0.1);

        if (!global_map || global_map->empty()) {
            LOG(WARNING) << "Global map is empty, skipping SendFullMap";
            return;
        }

        // 构造二进制数据包
        // 格式: [0..7] double timestamp | [8..11] uint32 point_count | [12..] point_count×16 bytes
        double timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        uint32_t point_count = global_map->size();

        // 计算总大小
        constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t);  // 12 bytes
        constexpr size_t POINT_SIZE = 4 * sizeof(float);                   // 16 bytes
        size_t total_size = HEADER_SIZE + point_count * POINT_SIZE;

        // 分配缓冲区
        std::vector<uint8_t> buffer(total_size);
        uint8_t* ptr = buffer.data();

        // 写入头部
        std::memcpy(ptr, &timestamp, sizeof(double));
        ptr += sizeof(double);
        std::memcpy(ptr, &point_count, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        // 写入点云数据
        for (const auto& pt : global_map->points) {
            float vals[4] = {pt.x, pt.y, pt.z, pt.intensity};
            std::memcpy(ptr, vals, POINT_SIZE);
            ptr += POINT_SIZE;
        }

        // 发送到 DORA
        std::string output_id = "full_map";
        dora_send_output(dora_context_,
                        const_cast<char*>(output_id.c_str()), output_id.length(),
                        reinterpret_cast<char*>(buffer.data()),
                        buffer.size());

        LOG(INFO) << "Sent full map with " << point_count << " points";
    }

    // ---------------------------------------------------------------------------
    // SendIncrementalMap - 发送增量地图点云（可选，用于实时可视化）
    // ---------------------------------------------------------------------------
    void SendIncrementalMap() {
        if (!dora_context_ || !slam_) return;

        auto lio = slam_->GetLIO();
        if (!lio) return;

        // 获取最新的去畸变点云（在世界坐标系下）
        auto recent_cloud = lio->GetRecentCloud();

        if (!recent_cloud || recent_cloud->empty()) {
            return;
        }

        // 构造二进制数据包
        // 格式: [0..7] double timestamp | [8..11] uint32 point_count | [12..] point_count×16 bytes
        double timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        uint32_t point_count = recent_cloud->size();

        // 计算总大小
        constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t);  // 12 bytes
        constexpr size_t POINT_SIZE = 4 * sizeof(float);                   // 16 bytes
        size_t total_size = HEADER_SIZE + point_count * POINT_SIZE;

        // 分配缓冲区
        std::vector<uint8_t> buffer(total_size);
        uint8_t* ptr = buffer.data();

        // 写入头部
        std::memcpy(ptr, &timestamp, sizeof(double));
        ptr += sizeof(double);
        std::memcpy(ptr, &point_count, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        // 写入点云数据
        for (const auto& pt : recent_cloud->points) {
            float vals[4] = {pt.x, pt.y, pt.z, pt.intensity};
            std::memcpy(ptr, vals, POINT_SIZE);
            ptr += POINT_SIZE;
        }

        // 发送到 DORA（使用 "pointcloud" 话题，用于实时可视化）
        std::string output_id = "pointcloud";
        dora_send_output(dora_context_,
                        const_cast<char*>(output_id.c_str()), output_id.length(),
                        reinterpret_cast<char*>(buffer.data()),
                        buffer.size());

        LOG(INFO) << "Sent full map with " << point_count << " points";
    }

    std::string yaml_path_;
    void* dora_context_;
    std::shared_ptr<SlamSystem> slam_;
    std::shared_ptr<PointCloudPreprocessPCL> preprocess_pcl_;
    int lidar_frame_count_ = 0;  // 用于控制完整地图发送频率
};

int main(int argc, char** argv) {
    std::cout << "Lightning-LM SLAM node for DORA" << std::endl;

    auto dora_context = init_dora_context_from_env();

    // 硬编码配置文件路径（相对于 NavigationFramework 根目录）
    std::string config_path = "../modules/mapping/lightning_lm_mapping/config/lightning_slam_config.yaml";
    std::cout << "Using config: " << config_path << std::endl;

    // 检查配置文件是否存在
    std::ifstream check_file(config_path);
    if (!check_file.good()) {
        std::cerr << "ERROR: Config file not found: " << config_path << std::endl;
        std::cerr << "Please run from NavigationFramework root directory" << std::endl;
        return -1;
    }
    check_file.close();

    DoraSlamNode lightning_node(config_path.c_str(), dora_context);
    lightning_node.Run(dora_context);

    // 无论事件循环如何退出，都保存地图
    printf("[lightning-lm] saving map before exit...\n");
    lightning_node.SaveMap();
    printf("[lightning-lm] map saved successfully\n");

    free_dora_context(dora_context);

    std::cout << "Lightning-LM SLAM node finished successfully" << std::endl;
    return 0;
}
