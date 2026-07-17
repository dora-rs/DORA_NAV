#pragma once

#include <vector>
#include <utility>
#include <cmath>

namespace pure_pursuit {

// =====================================================================
//  二维位姿
// =====================================================================
struct Pose2D {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

// =====================================================================
//  算法配置参数
//  说明：所有参数均可在节点层按场景覆盖，此处为合理默认值
// =====================================================================
struct PurePursuitConfig {
    // --- 前视距离 ---
    double k      = 0.3;    // 速度-前视增益 Lf = k * v + Lfc
    double Lfc    = 0.35;   // 基础前视距离 [m]
    double Lf_min = 0.35;   // 前视距离下限 [m]
    double Lf_max = 1.5;    // 前视距离上限 [m]

    // --- 速度 ---
    double min_velocity     = 0.08;  // 最低线速度 [m/s]
    double max_velocity     = 0.18;  // 最高线速度 [m/s]
    double max_angular_vel  = 0.8;   // 最大角速度 [rad/s]

    // --- 曲率映射到速度 ---
    double min_curvature = 0.0;
    double max_curvature = 3.0;

    // --- 终点判断 ---
    double goal_threshold = 0.40;   // 距终点小于此值视为到达 [m]

    // --- 近点搜索窗口 ---
    int nearest_search_back = 20;   // calcNearestIndexIncremental 向前回溯的点数

    // --- 初始对齐阶段（原地旋转） ---
    double align_enter_threshold = 45.0 * M_PI / 180.0; // 进入对齐阶段的角度阈值 [rad]（45°）
    double align_exit_threshold  = 10.0 * M_PI / 180.0; // 退出对齐阶段的角度阈值 [rad]（10°）
    double align_kp              = 1.2;                  // 对齐阶段角速度 P 增益
    double align_angular_vel_max = 0.6;                  // 对齐阶段角速度上限 [rad/s]
    int    align_path_points     = 5;                    // 计算路径初始朝向所用的前 N 个点

    // --- 终点 yaw 对齐阶段 ---
    double goal_yaw_exit_threshold  = 5.0 * M_PI / 180.0; // 终点 yaw 对齐完成阈值 [rad]（5°）
    double goal_yaw_kp              = 1.2;                 // 终点对齐 P 增益（同起始对齐）
    double goal_yaw_angular_vel_max = 0.4;                 // 终点对齐角速度上限 [rad/s]（慢一些，更精准）
    bool   enable_goal_yaw_align    = true;                // 是否启用终点 yaw 对齐
};

// =====================================================================
//  Pure Pursuit 核心算法类
//
//  对比 v1 的改进点：
//  1. 前视距离自适应（速度线性 + min/max 截断）
//  2. 速度-曲率映射改用线性插值，更直观可调
//  3. 角速度限幅独立于线速度，避免低速时过大角速度
//  4. 路径更新支持重置（新 goal 到来可重新跟踪）
//  5. 公共接口更简洁，状态完全封装
// =====================================================================
class PurePursuitComponent {
public:
    explicit PurePursuitComponent(const PurePursuitConfig& cfg = PurePursuitConfig{});

    // 重置内部状态（新路径到来时调用）
    void reset();

    // 主计算接口：输入路径 + 位姿 + 速度，返回 {linear_x, angular_z}
    std::pair<double, double> compute(
        const std::vector<double>& cx,
        const std::vector<double>& cy,
        const std::vector<double>& cyaw,
        const std::vector<double>& ck,
        const Pose2D& pose,
        double velocity);

    // 是否已到达终点（位置 + yaw 均完成）
    bool isGoalReached() const { return goal_reached_; }

    // 是否正在执行终点 yaw 对齐（供外部监控用）
    bool isGoalYawAligning() const { return goal_yaw_aligning_; }

    // 设置目标 yaw（新 goal 到来时由 Dora 节点调用）
    // has_yaw=false 时不执行终点对齐
    void setGoalYaw(double yaw, bool has_yaw = true) {
        goal_yaw_       = yaw;
        has_goal_yaw_   = has_yaw;
    }

private:
    // 自适应前视距离
    double calcLookahead(double velocity) const;

    // 搜索前视目标点索引，返回 {index, Lf}
    std::pair<int, double> searchTargetIndex(
        const std::vector<double>& cx,
        const std::vector<double>& cy,
        double Lf);

    // 首次搜索最近点（全局遍历）
    int calcFirstNearestIndex(
        const std::vector<double>& cx,
        const std::vector<double>& cy) const;

    // 增量搜索最近点（局部窗口）
    int calcNearestIndexIncremental(
        const std::vector<double>& cx,
        const std::vector<double>& cy) const;

    // 曲率 → 目标线速度（线性映射）
    double curvatureToVelocity(double curvature) const;

    // 将 alpha 规范化到 (-π, π]，并处理边界抖动
    static double normalizeAlpha(double alpha);

    // 欧氏距离
    static double dist(double x1, double y1, double x2, double y2);

    // 计算路径起始朝向（取最近点后 N 个点的整体方向）
    double calcPathStartYaw(
        const std::vector<double>& cx,
        const std::vector<double>& cy,
        int start_idx) const;

    // 对齐阶段：原地旋转，返回 {0.0, w}
    std::pair<double, double> computeAlign(double heading_error,
                                           double kp, double max_w);

    // ---- 成员变量 ----
    PurePursuitConfig cfg_;

    Pose2D  pose_;
    double  velocity_   = 0.0;

    int     nearest_idx_    = -1;   // 上一帧最近点索引
    bool    path_init_      = false;
    bool    goal_reached_   = false;

    // --- 起始对齐阶段状态 ---
    bool    aligning_           = false; // 当前是否处于起始原地对齐阶段

    // --- 终点 yaw 对齐阶段状态 ---
    bool    goal_yaw_aligning_  = false; // 当前是否处于终点 yaw 对齐阶段
    double  goal_yaw_           = 0.0;   // 目标 yaw 值 [rad]
    bool    has_goal_yaw_       = false; // 目标是否携带 yaw 信息
};

} // namespace pure_pursuit
