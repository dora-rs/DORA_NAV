/**
 * dwa_planner_dora_node.cpp
 *
 * DWA 局部规划器 Dora 节点
 *
 * 输入 (来自 run.yml):
 *   timer      ← dora/timer/millis/50  (控制周期触发)
 *   path       ← astar_planner/path
 *              格式: { "header":{...}, "poses":[{"x":...,"y":...}, ...] }
 *   pose       ← hdl_localization/pose
 *              格式: { "pose":{ "position":{x,y,z}, "orientation":{x,y,z,w} } }
 *   pointcloud ← livox_driver/pointcloud
 *              格式: 二进制 [timestamp(8B) + num_points(4B) + N×(x,y,z,intensity)(16B)]
 *   Odometry   ← mickrobotx4/Odometry
 *              格式: { "twist":{ "linear":{x,y,z}, "angular":{x,y,z} } }
 *
 * 输出:
 *   cmd_vel    → mickrobotx4/cmd_vel
 *              格式: { "linear":{"x":...}, "angular":{"z":...} }
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <chrono>

#include <nlohmann/json.hpp>
#include "dwa_planner/types.hpp"
#include "dwa_planner/pointcloud_processor.hpp"
#include "dwa_planner/costmap_builder.hpp"
#include "dwa_planner/dwa_planner_component.hpp"

extern "C" {
#include "node_api.h"
}

using json = nlohmann::json;
using namespace dwa_planner;

// =====================================================================
//  辅助：四元数 → yaw(弧度制)
// =====================================================================
static double quatToYaw(double qx, double qy, double qz, double qw) {
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
}

// =====================================================================
//  辅助：全局路径转换到机器人坐标系
// =====================================================================
static void transformPathToRobotFrame(
    const std::vector<double>& global_path_x,
    const std::vector<double>& global_path_y,
    const Pose2D& robot_pose,
    std::vector<double>& local_path_x,
    std::vector<double>& local_path_y)
{
    local_path_x.clear();
    local_path_y.clear();

    double cos_yaw = std::cos(robot_pose.yaw);
    double sin_yaw = std::sin(robot_pose.yaw);

    for (size_t i = 0; i < global_path_x.size(); ++i) {
        // 平移到机器人位置
        double dx = global_path_x[i] - robot_pose.x;
        double dy = global_path_y[i] - robot_pose.y;

        // 旋转到机器人朝向
        double local_x = cos_yaw * dx + sin_yaw * dy;
        double local_y = -sin_yaw * dx + cos_yaw * dy;

        local_path_x.push_back(local_x);
        local_path_y.push_back(local_y);
    }
}

// =====================================================================
//  Dora 节点类
// =====================================================================
class DWAPlannerDoraNode {
public:
    explicit DWAPlannerDoraNode(void* dora_context)
        : dora_context_(dora_context),
          pointcloud_processor_(pc_config_),
          costmap_builder_(costmap_config_),
          dwa_planner_(dwa_config_) {}

    void run() {
        std::cout << "[DWAPlanner] event loop started" << std::endl;

        while (true) {
            void* event = dora_next_event(dora_context_);
            if (!event) {
                std::cerr << "[DWAPlanner] unexpected end of dora events" << std::endl;
                break;
            }

            DoraEventType type = read_dora_event_type(event);

            if (type == DoraEventType_Input) {
                char*  data     = nullptr;
                size_t data_len = 0;
                char*  id       = nullptr;
                size_t id_len   = 0;

                read_dora_input_data(event, &data, &data_len);
                read_dora_input_id(event, &id, &id_len);

                std::string topic(id, id_len);

                if (topic == "path") {
                    handlePath(data, data_len);
                } else if (topic == "pose") {
                    handlePose(data, data_len);
                } else if (topic == "pointcloud") {
                    handlePointCloud(data, data_len);
                } else if (topic == "Odometry") {
                    handleOdometry(data, data_len);
                } else if (topic == "timer") {
                    handleTimer();
                }
            } else if (type == DoraEventType_Stop) {
                std::cout << "[DWAPlanner] stop event received" << std::endl;
                free_dora_event(event);
                break;
            }

            free_dora_event(event);
        }
    }

private:
    // ------------------------------------------------------------------
    //  输入处理：path
    //  格式: { "header":{...}, "poses":[{"x":...,"y":...}, ...] }
    // ------------------------------------------------------------------
    void handlePath(const char* data, size_t len) {
        try {
            json j = json::parse(std::string(data, len));

            if (!j.contains("poses") || j["poses"].empty()) {
                std::cerr << "[DWAPlanner] received empty path, ignored" << std::endl;
                return;
            }

            path_x_.clear();
            path_y_.clear();

            for (const auto& pt : j["poses"]) {
                path_x_.push_back(pt["x"].get<double>());
                path_y_.push_back(pt["y"].get<double>());
            }

            path_received_ = true;

            std::cout << "[DWAPlanner] new path received, "
                      << path_x_.size() << " waypoints" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[DWAPlanner] path parse error: " << e.what() << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  输入处理：pose
    //  格式: { "pose":{ "position":{x,y,z}, "orientation":{x,y,z,w} } }
    // ------------------------------------------------------------------
    void handlePose(const char* data, size_t len) {
        try {
            json j = json::parse(std::string(data, len));

            current_state_.pose.x = j["pose"]["position"]["x"].get<double>();
            current_state_.pose.y = j["pose"]["position"]["y"].get<double>();

            double qx = j["pose"]["orientation"]["x"].get<double>();
            double qy = j["pose"]["orientation"]["y"].get<double>();
            double qz = j["pose"]["orientation"]["z"].get<double>();
            double qw = j["pose"]["orientation"]["w"].get<double>();
            current_state_.pose.yaw = quatToYaw(qx, qy, qz, qw);

            pose_received_ = true;
        } catch (const std::exception& e) {
            std::cerr << "[DWAPlanner] pose parse error: " << e.what() << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  输入处理：pointcloud（二进制格式）
    //  格式: [timestamp(8B) + num_points(4B) + N×(x,y,z,intensity)(16B)]
    //  只提取xyz坐标，忽略时间戳和强度
    // ------------------------------------------------------------------
    void handlePointCloud(const char* data, size_t len) {
        try {
            const char* ptr = data;

            // 跳过timestamp
            ptr += sizeof(double);

            // 读取点数
            uint32_t num_points;
            std::memcpy(&num_points, ptr, sizeof(uint32_t));
            ptr += sizeof(uint32_t);

            // 只提取xyz坐标到Eigen矩阵
            pointcloud_.points.resize(num_points, 3);

            for (uint32_t i = 0; i < num_points; ++i) {
                float vals[4];
                std::memcpy(vals, ptr, 4 * sizeof(float));
                ptr += 4 * sizeof(float);

                pointcloud_.points(i, 0) = vals[0];  // x
                pointcloud_.points(i, 1) = vals[1];  // y
                pointcloud_.points(i, 2) = vals[2];  // z
                // vals[3] (intensity) 被忽略
            }

            pointcloud_received_ = true;

            // 首次接收时打印信息
            static bool first_cloud = true;
            if (first_cloud) {
                std::cout << "[DWAPlanner] first pointcloud received: "
                          << num_points << " points" << std::endl;
                first_cloud = false;
            }
        } catch (const std::exception& e) {
            std::cerr << "[DWAPlanner] pointcloud parse error: " << e.what() << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  输入处理：Odometry
    //  格式: { "twist":{ "linear":{x,y,z}, "angular":{x,y,z} } }
    // ------------------------------------------------------------------
    void handleOdometry(const char* data, size_t len) {
        try {
            json j = json::parse(std::string(data, len));
            current_state_.vx    = j["twist"]["linear"]["x"].get<double>();
            current_state_.omega = j["twist"]["angular"]["z"].get<double>();
            odometry_received_ = true;
        } catch (const std::exception& e) {
            std::cerr << "[DWAPlanner] Odometry parse error: " << e.what() << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  定时器触发：执行控制计算并发布 cmd_vel
    // ------------------------------------------------------------------
    void handleTimer() {
        // 检查所有数据是否就绪
        if (!path_received_ || !pose_received_ ||
            !pointcloud_received_ || !odometry_received_) {
            return;
        }

        // 首次所有数据就绪时打印状态
        static bool first_ready = true;
        if (first_ready) {
            std::cout << "[DWAPlanner] all data ready, starting control loop" << std::endl;
            std::cout << "  - Path: " << path_x_.size() << " waypoints" << std::endl;
            std::cout << "  - Pose: (" << current_state_.pose.x << ", "
                      << current_state_.pose.y << ", yaw="
                      << current_state_.pose.yaw * 180.0 / M_PI << "°)" << std::endl;
            std::cout << "  - PointCloud: " << pointcloud_.size() << " points" << std::endl;
            std::cout << "  - Velocity: vx=" << current_state_.vx
                      << " m/s, omega=" << current_state_.omega << " rad/s" << std::endl;
            first_ready = false;
        }

        // 处理点云：降采样 → 高度过滤 → 距离过滤
        // 注意：点云数据已经在机器人坐标系下
        auto start_time = std::chrono::high_resolution_clock::now();

        auto processed_cloud = pointcloud_processor_.process(pointcloud_.points);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        // 构建局部代价地图
        auto costmap_start = std::chrono::high_resolution_clock::now();

        costmap_builder_.buildFromPointCloud(processed_cloud);

        auto costmap_end = std::chrono::high_resolution_clock::now();
        auto costmap_duration = std::chrono::duration_cast<std::chrono::microseconds>(costmap_end - costmap_start);

        // 打印处理统计（每10帧打印一次，避免刷屏）
        static int frame_count = 0;
        // if (++frame_count % 10 == 0) {
        //     std::cout << "[DWAPlanner] Processing: "
        //               << pointcloud_.size() << " → " << processed_cloud.rows() << " points, "
        //               << "PC: " << duration.count() / 1000.0 << " ms, "
        //               << "Costmap: " << costmap_duration.count() / 1000.0 << " ms" << std::endl;
        // }

        // 保存代价地图用于可视化验证（每100帧保存一次）
        // static int save_count = 0;
        // if (++save_count % 100 == 0) {
        //     costmap_builder_.saveToPGM("costmap_debug.pgm");
        // }

        // 将全局路径转换到机器人坐标系
        std::vector<double> local_path_x, local_path_y;
        transformPathToRobotFrame(path_x_, path_y_, current_state_.pose, local_path_x, local_path_y);

        // // 打印当前里程计速度（每10帧打印一次）
        // if (frame_count % 10 == 0) {
        //     std::cout << "[DWAPlanner] Odometry: vx=" << current_state_.vx
        //               << " m/s, omega=" << current_state_.omega << " rad/s" << std::endl;
        // }

        // 调用DWA算法计算最优速度（使用局部坐标系路径）
        auto dwa_start = std::chrono::high_resolution_clock::now();

        auto [v, w] = dwa_planner_.compute(current_state_, local_path_x, local_path_y, costmap_builder_);

        auto dwa_end = std::chrono::high_resolution_clock::now();
        auto dwa_duration = std::chrono::duration_cast<std::chrono::microseconds>(dwa_end - dwa_start);

        // // 打印DWA计算耗时（每10帧打印一次）
        // if (frame_count % 10 == 0) {
        //     std::cout << "[DWAPlanner] DWA compute: " << dwa_duration.count() / 1000.0 << " ms, "
        //               << "cmd_vel: v=" << v << " m/s, w=" << w << " rad/s" << std::endl;
        // }

        // 发布所有预测轨迹用于可视化
        publishTrajectories(dwa_planner_.getAllTrajectories());

        // 发送速度指令
        publishCmdVel(v, w);
    }

    // ------------------------------------------------------------------
    //  输出：cmd_vel
    //  格式: { "linear":{"x": v}, "angular":{"z": w} }
    // ------------------------------------------------------------------
    void publishCmdVel(double linear_x, double angular_z) {
        json j;
        j["linear"]["x"]  = linear_x;
        j["angular"]["z"] = angular_z;

        std::string payload = j.dump();
        const std::string topic = "cmd_vel";

        int rc = dora_send_output(
            dora_context_,
            const_cast<char*>(topic.c_str()), topic.length(),
            const_cast<char*>(payload.c_str()), payload.length());

        if (rc != 0) {
            std::cerr << "[DWAPlanner] dora_send_output failed (rc=" << rc << ")" << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  输出：dwa_trajectories（所有预测轨迹，全局坐标系，二进制格式）
    //  格式: [num_trajectories(uint32_t)] +
    //        N × [num_points(uint32_t) + M × (x(float), y(float))]
    // ------------------------------------------------------------------
    void publishTrajectories(const std::vector<std::vector<std::pair<float, float>>>& local_trajectories) {
        double cos_yaw = std::cos(current_state_.pose.yaw);
        double sin_yaw = std::sin(current_state_.pose.yaw);

        std::vector<uint8_t> buffer;

        // 写入轨迹数量
        uint32_t num_trajectories = static_cast<uint32_t>(local_trajectories.size());
        buffer.insert(buffer.end(),
                     reinterpret_cast<uint8_t*>(&num_trajectories),
                     reinterpret_cast<uint8_t*>(&num_trajectories) + sizeof(uint32_t));

        // 写入每条轨迹
        for (const auto& local_traj : local_trajectories) {
            uint32_t num_points = static_cast<uint32_t>(local_traj.size());
            buffer.insert(buffer.end(),
                         reinterpret_cast<uint8_t*>(&num_points),
                         reinterpret_cast<uint8_t*>(&num_points) + sizeof(uint32_t));

            for (const auto& pt : local_traj) {
                // 从机器人坐标系转换到全局坐标系
                float global_x = static_cast<float>(current_state_.pose.x + cos_yaw * pt.first - sin_yaw * pt.second);
                float global_y = static_cast<float>(current_state_.pose.y + sin_yaw * pt.first + cos_yaw * pt.second);

                buffer.insert(buffer.end(),
                             reinterpret_cast<uint8_t*>(&global_x),
                             reinterpret_cast<uint8_t*>(&global_x) + sizeof(float));
                buffer.insert(buffer.end(),
                             reinterpret_cast<uint8_t*>(&global_y),
                             reinterpret_cast<uint8_t*>(&global_y) + sizeof(float));
            }
        }

        const std::string topic = "dwa_trajectories";
        int rc = dora_send_output(
            dora_context_,
            const_cast<char*>(topic.c_str()), topic.length(),
            reinterpret_cast<char*>(buffer.data()), buffer.size());

        if (rc != 0) {
            std::cerr << "[DWAPlanner] dora_send_output (trajectories) failed (rc=" << rc << ")" << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  成员变量
    // ------------------------------------------------------------------
    void* dora_context_;

    // 路径数据
    std::vector<double> path_x_, path_y_;
    bool path_received_ = false;

    // 当前位姿和速度（来自 hdl_localization 和 Odometry）
    RobotState current_state_{};
    bool pose_received_     = false;
    bool odometry_received_ = false;

    // 点云数据（来自 livox_driver）
    PointCloud pointcloud_;
    bool pointcloud_received_ = false;

    // 点云处理器
    PointCloudProcessorConfig pc_config_;
    PointCloudProcessor pointcloud_processor_;

    // 代价地图构建器
    CostmapConfig costmap_config_;
    CostmapBuilder costmap_builder_;

    // DWA规划器
    DWAConfig dwa_config_;
    DWAPlannerComponent dwa_planner_;
};

// =====================================================================
//  main
// =====================================================================
int main() {
    std::cout << "[DWAPlanner] Dora node starting..." << std::endl;

    void* dora_context = init_dora_context_from_env();
    if (!dora_context) {
        std::cerr << "[DWAPlanner] failed to initialize dora context" << std::endl;
        return 1;
    }

    DWAPlannerDoraNode node(dora_context);
    node.run();

    free_dora_context(dora_context);
    std::cout << "[DWAPlanner] Dora node exiting" << std::endl;
    return 0;
}
