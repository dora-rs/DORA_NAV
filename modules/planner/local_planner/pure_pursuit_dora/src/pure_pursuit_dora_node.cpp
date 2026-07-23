/**
 * pure_pursuit_dora_node.cpp
 *
 * Pure Pursuit 局部规划器 Dora 节点
 *
 * 输入 (来自 run.yml):
 *   path      ← astar_planner/path
 *              格式: { "header":{...}, "poses":[{"x":...,"y":...}, ...] }
 *   goal      ← goal_publisher/goal
 *              格式: { "x":..., "y":..., "yaw":... }   （yaw 可选）
 *   pose      ← hdl_localization/pose
 *              格式: { "pose":{ "position":{x,y,z}, "orientation":{x,y,z,w} } }
 *   Odometry  ← mickrobotx4/Odometry
 *              格式: { "twist":{ "linear":{x,y,z}, "angular":{x,y,z} } }
 *   timer     ← dora/timer/millis/10  (控制周期触发)
 *
 * 输出:
 *   cmd_vel   → mickrobotx4/cmd_vel
 *              格式: { "linear":{"x":...}, "angular":{"z":...} }
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <optional>

#include <nlohmann/json.hpp>
#include "pure_pursuit/pure_pursuit_component.hpp"

extern "C" {
#include "node_api.h"
}

using json = nlohmann::json;
using namespace pure_pursuit;

// =====================================================================
//  辅助：四元数 → yaw
// =====================================================================
static double quatToYaw(double qx, double qy, double qz, double qw) {
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
}

// =====================================================================
//  辅助：路径预处理（计算 yaw 序列 + Menger 曲率序列）
//  与 v1 完全相同，保证与 astar_planner 输出格式兼容
// =====================================================================
static void computePathYawAndCurvature(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    std::vector<double>& cyaw,
    std::vector<double>& ck)
{
    const int n = static_cast<int>(cx.size());
    cyaw.assign(n, 0.0);
    ck.assign(n, 0.0);

    if (n == 0) return;

    // yaw：相邻点方向
    for (int i = 0; i < n - 1; ++i) {
        cyaw[i] = std::atan2(cy[i + 1] - cy[i], cx[i + 1] - cx[i]);
    }
    cyaw[n - 1] = cyaw[std::max(0, n - 2)];

    if (n < 3) return;

    // Menger 曲率：三点圆弧法
    for (int i = 1; i < n - 1; ++i) {
        double ax = cx[i]     - cx[i - 1], ay = cy[i]     - cy[i - 1];
        double bx = cx[i + 1] - cx[i],     by = cy[i + 1] - cy[i];
        double cross = ax * by - ay * bx;
        double la = std::hypot(ax, ay);
        double lb = std::hypot(bx, by);
        double lc = std::hypot(cx[i + 1] - cx[i - 1], cy[i + 1] - cy[i - 1]);
        ck[i] = (la * lb * lc < 1e-9) ? 0.0 : 2.0 * std::abs(cross) / (la * lb * lc);
    }
    ck[0]     = ck[1];
    ck[n - 1] = ck[n - 2];
}

// =====================================================================
//  Dora 节点类
// =====================================================================
class PurePursuitDoraNode {
public:
    explicit PurePursuitDoraNode(void* dora_context)
        : dora_context_(dora_context), planner_(config_) {}

    void run() {
        std::cout << "[PurePursuit] event loop started" << std::endl;

        while (true) {
            void* event = dora_next_event(dora_context_);
            if (!event) {
                std::cerr << "[PurePursuit] unexpected end of dora events" << std::endl;
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
                } else if (topic == "goal") {
                    handleGoal(data, data_len);
                } else if (topic == "pose") {
                    handlePose(data, data_len);
                } else if (topic == "Odometry") {
                    handleOdometry(data, data_len);
                } else if (topic == "timer") {
                    handleTimer();
                }
            } else if (type == DoraEventType_Stop) {
                std::cout << "[PurePursuit] stop event received" << std::endl;
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
    //  支持重复接收新路径（新 goal 触发后更新）
    // ------------------------------------------------------------------
    void handlePath(const char* data, size_t len) {
        try {
            json j = json::parse(std::string(data, len));

            if (!j.contains("poses") || j["poses"].empty()) {
                std::cerr << "[PurePursuit] received empty path, ignored" << std::endl;
                return;
            }

            cx_.clear();
            cy_.clear();

            for (const auto& pt : j["poses"]) {
                cx_.push_back(pt["x"].get<double>());
                cy_.push_back(pt["y"].get<double>());
            }

            computePathYawAndCurvature(cx_, cy_, cyaw_, ck_);

            // 重置算法内部状态，准备跟踪新路径
            planner_.reset();
            path_received_  = true;

            std::cout << "[PurePursuit] new path received, "
                      << cx_.size() << " waypoints" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[PurePursuit] path parse error: " << e.what() << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  输入处理：goal
    //  格式: { "x":..., "y":..., "yaw":... }   （yaw 为可选字段）
    //  只提取 yaw，位置部分由 astar_planner 处理后通过 path 传入
    // ------------------------------------------------------------------
    void handleGoal(const char* data, size_t len) {
        try {
            json j = json::parse(std::string(data, len));

            if (j.contains("yaw") && j["yaw"].is_number()) {
                double goal_yaw = j["yaw"].get<double>();
                planner_.setGoalYaw(goal_yaw, true);
                std::cout << "[PurePursuit] goal yaw updated: "
                          << goal_yaw * 180.0 / M_PI << " deg" << std::endl;
            } else {
                // 目标不含 yaw，禁用终点对齐
                planner_.setGoalYaw(0.0, false);
                std::cout << "[PurePursuit] goal has no yaw, skip goal yaw align" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "[PurePursuit] goal parse error: " << e.what() << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  输入处理：pose
    //  格式: { "pose":{ "position":{x,y,z}, "orientation":{x,y,z,w} } }
    // ------------------------------------------------------------------
    void handlePose(const char* data, size_t len) {
        try {
            json j = json::parse(std::string(data, len));

            current_pose_.x = j["pose"]["position"]["x"].get<double>();
            current_pose_.y = j["pose"]["position"]["y"].get<double>();

            double qx = j["pose"]["orientation"]["x"].get<double>();
            double qy = j["pose"]["orientation"]["y"].get<double>();
            double qz = j["pose"]["orientation"]["z"].get<double>();
            double qw = j["pose"]["orientation"]["w"].get<double>();
            current_pose_.yaw = quatToYaw(qx, qy, qz, qw);

            pose_received_ = true;
        } catch (const std::exception& e) {
            std::cerr << "[PurePursuit] pose parse error: " << e.what() << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  输入处理：Odometry（只取线速度 vx 用于自适应前视距离）
    //  格式: { "twist":{ "linear":{x,y,z}, "angular":{x,y,z} } }
    // ------------------------------------------------------------------
    void handleOdometry(const char* data, size_t len) {
        try {
            json j = json::parse(std::string(data, len));
            current_vx_        = j["twist"]["linear"]["x"].get<double>();
            velocity_received_ = true;
        } catch (const std::exception& e) {
            std::cerr << "[PurePursuit] Odometry parse error: " << e.what() << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  定时器触发：执行控制计算并发布 cmd_vel
    // ------------------------------------------------------------------
    void handleTimer() {
        // 三路数据均就绪才开始控制
        if (!path_received_ || !pose_received_ || !velocity_received_) return;

        // 已到达终点：持续发停车指令，不再计算
        if (planner_.isGoalReached()) {
            publishCmdVel(0.0, 0.0);
            return;
        }

        auto [v, w] = planner_.compute(
            cx_, cy_, cyaw_, ck_,
            current_pose_, current_vx_);

        publishCmdVel(v, w);
    }

    // ------------------------------------------------------------------
    //  输出：cmd_vel
    //  格式: { "linear":{"x": v}, "angular":{"z": w} }
    // ------------------------------------------------------------------
    void publishCmdVel(double linear_x, double angular_z) {
        // std::cout << "[PurePursuit] linear_x=" << linear_x
        //           << "  angular_z=" << angular_z << std::endl;

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
            std::cerr << "[PurePursuit] dora_send_output failed (rc=" << rc << ")" << std::endl;
        }
    }

    // ------------------------------------------------------------------
    //  成员变量
    // ------------------------------------------------------------------
    void* dora_context_;

    PurePursuitConfig    config_;
    PurePursuitComponent planner_;

    // 路径数据
    std::vector<double> cx_, cy_, cyaw_, ck_;
    bool path_received_ = false;

    // 当前位姿（来自 hdl_localization）
    Pose2D current_pose_{};
    bool   pose_received_ = false;

    // 当前线速度（来自 Odometry）
    double current_vx_        = 0.0;
    bool   velocity_received_ = false;
};

// =====================================================================
//  main
// =====================================================================
int main() {
    std::cout << "[PurePursuit] Dora node starting..." << std::endl;

    void* dora_context = init_dora_context_from_env();
    if (!dora_context) {
        std::cerr << "[PurePursuit] failed to initialize dora context" << std::endl;
        return 1;
    }

    PurePursuitDoraNode node(dora_context);
    node.run();

    free_dora_context(dora_context);
    std::cout << "[PurePursuit] Dora node exiting" << std::endl;
    return 0;
}
