#include "pure_pursuit/pure_pursuit_component.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace pure_pursuit {

// =====================================================================
//  构造 / 重置
// =====================================================================
PurePursuitComponent::PurePursuitComponent(const PurePursuitConfig& cfg)
    : cfg_(cfg) {}

void PurePursuitComponent::reset() {
    nearest_idx_       = -1;
    path_init_         = false;
    goal_reached_      = false;
    velocity_          = 0.0;
    aligning_          = false;   // 新路径到来时清除起始对齐状态
    goal_yaw_aligning_ = false;   // 新路径到来时清除终点对齐状态
}

// =====================================================================
//  主计算入口
// =====================================================================
std::pair<double, double> PurePursuitComponent::compute(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    const std::vector<double>& ck,
    const Pose2D& pose,
    double velocity)
{
    if (cx.empty()) {
        std::cerr << "[PurePursuit] path is empty, stopping." << std::endl;
        return {0.0, 0.0};
    }

    pose_     = pose;
    velocity_ = velocity;

    // ---- 1. 终点位置检测 ----
    const size_t last = cx.size() - 1;
    double goal_dist = dist(pose_.x, pose_.y, cx[last], cy[last]);
    if (goal_dist < cfg_.goal_threshold || goal_yaw_aligning_) {

        // ---- 1a. 终点 yaw 对齐阶段 ----
        //  位置到达后，若目标携带 yaw 且未完成对齐，进入原地旋转对齐
        if (cfg_.enable_goal_yaw_align && has_goal_yaw_) {
            double yaw_error = normalizeAlpha(goal_yaw_ - pose_.yaw);

            if (!goal_yaw_aligning_) {
                // 首次触发：检查是否需要对齐
                if (std::abs(yaw_error) > cfg_.goal_yaw_exit_threshold) {
                    goal_yaw_aligning_ = true;
                    std::cout << "[PurePursuit] position reached, enter GOAL_YAW align, "
                              << "error=" << yaw_error * 180.0 / M_PI << " deg" << std::endl;
                } else {
                    // yaw 已经满足，直接完成
                    goal_reached_ = true;
                    std::cout << "[PurePursuit] goal reached (dist=" << goal_dist
                              << "m, yaw_error=" << yaw_error * 180.0 / M_PI << " deg, skip align)" << std::endl;
                    return {0.0, 0.0};
                }
            }

            if (goal_yaw_aligning_) {
                yaw_error = normalizeAlpha(goal_yaw_ - pose_.yaw);  // 重新计算（每帧更新）
                if (std::abs(yaw_error) < cfg_.goal_yaw_exit_threshold) {
                    // 对齐完成
                    goal_yaw_aligning_ = false;
                    goal_reached_      = true;
                    std::cout << "[PurePursuit] GOAL_YAW align done, goal fully reached" << std::endl;
                    return {0.0, 0.0};
                }
                // 继续旋转对齐
                return computeAlign(yaw_error, cfg_.goal_yaw_kp, cfg_.goal_yaw_angular_vel_max);
            }
        }

        // ---- 1b. 无 yaw 要求或对齐已完成：直接到达 ----
        goal_reached_ = true;
        std::cout << "[PurePursuit] goal reached (dist=" << goal_dist << "m)" << std::endl;
        return {0.0, 0.0};
    }

    // ---- 2. 确定最近点（对齐阶段也需要用来计算路径朝向） ----
    if (!path_init_ || nearest_idx_ < 0) {
        nearest_idx_ = calcFirstNearestIndex(cx, cy);
        path_init_   = true;
    } else {
        int updated = calcNearestIndexIncremental(cx, cy);
        if (updated >= 0) nearest_idx_ = updated;
        if (nearest_idx_ < 0) nearest_idx_ = calcFirstNearestIndex(cx, cy);
    }

    // ---- 3. 初始对齐阶段检测 ----
    //  取最近点起始 N 个路径点的整体朝向作为对齐目标
    double path_yaw     = calcPathStartYaw(cx, cy, nearest_idx_);
    double heading_error = normalizeAlpha(path_yaw - pose_.yaw);

    //  进入条件：角度偏差超过 enter_threshold（45°）
    if (!aligning_ && std::abs(heading_error) > cfg_.align_enter_threshold) {
        aligning_ = true;
        std::cout << "[PurePursuit] enter ALIGN phase, heading_error="
                  << heading_error * 180.0 / M_PI << " deg" << std::endl;
    }

    //  处于对齐阶段：原地旋转，不输出线速度
    if (aligning_) {
        //  退出条件：角度误差收敛到 exit_threshold（10°）以内
        if (std::abs(heading_error) < cfg_.align_exit_threshold) {
            aligning_ = false;
            std::cout << "[PurePursuit] exit ALIGN phase, switching to TRACK" << std::endl;
        } else {
            return computeAlign(heading_error, cfg_.align_kp, cfg_.align_angular_vel_max);
        }
    }

    // ---- 4. 自适应前视距离 ----
    double Lf = calcLookahead(velocity_);

    // ---- 5. 搜索前视目标点（nearest_idx_ 已在 Step 2 更新） ----
    int idx = nearest_idx_;
    const int n = static_cast<int>(cx.size());
    while (idx < n - 1 && dist(pose_.x, pose_.y, cx[idx], cy[idx]) < Lf) {
        ++idx;
    }
    int target_idx = idx;

    // ---- 6. 计算 alpha（车辆坐标系下目标方位角） ----
    double dx    = cx[target_idx] - pose_.x;
    double dy    = cy[target_idx] - pose_.y;
    double alpha = std::atan2(dy, dx) - pose_.yaw;
    alpha = normalizeAlpha(alpha);

    // ---- 7. 目标点处曲率 → 目标线速度 ----
    double raw_curvature = std::abs(ck[target_idx]);
    double v_cmd = curvatureToVelocity(raw_curvature);
    v_cmd = std::clamp(v_cmd, cfg_.min_velocity, cfg_.max_velocity);

    // ---- 8. 纯跟踪角速度：w = 2*v*sin(alpha)/Lf ----
    double w_cmd = 2.0 * v_cmd * std::sin(alpha) / Lf;
    w_cmd = std::clamp(w_cmd, -cfg_.max_angular_vel, cfg_.max_angular_vel);

    return {v_cmd, w_cmd};
}

// =====================================================================
//  自适应前视距离
// =====================================================================
double PurePursuitComponent::calcLookahead(double velocity) const {
    double Lf = cfg_.k * std::abs(velocity) + cfg_.Lfc;
    return std::clamp(Lf, cfg_.Lf_min, cfg_.Lf_max);
}

// =====================================================================
//  搜索前视目标点（保留供外部单独调用，compute 内已内联）
//  策略：先定最近点（增量或全局），再向前找第一个距离 >= Lf 的点
// =====================================================================
std::pair<int, double> PurePursuitComponent::searchTargetIndex(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    double Lf)
{
    // 首次或 reset 后全局搜索最近点
    if (!path_init_ || nearest_idx_ < 0) {
        nearest_idx_ = calcFirstNearestIndex(cx, cy);
        path_init_   = true;
    } else {
        int updated = calcNearestIndexIncremental(cx, cy);
        if (updated >= 0) {
            nearest_idx_ = updated;
        }
        if (nearest_idx_ < 0) {
            nearest_idx_ = calcFirstNearestIndex(cx, cy);
        }
    }

    // 从最近点出发向前搜索，找到第一个满足 dist >= Lf 的点
    int idx = nearest_idx_;
    const int n = static_cast<int>(cx.size());
    while (idx < n - 1 &&
           dist(pose_.x, pose_.y, cx[idx], cy[idx]) < Lf) {
        ++idx;
    }

    return {idx, Lf};
}

// =====================================================================
//  全局最近点搜索（O(n)，仅在初始化时调用）
// =====================================================================
int PurePursuitComponent::calcFirstNearestIndex(
    const std::vector<double>& cx,
    const std::vector<double>& cy) const
{
    double min_dist = std::numeric_limits<double>::max();
    int    min_idx  = 0;
    for (int i = 0; i < static_cast<int>(cx.size()); ++i) {
        double d = dist(pose_.x, pose_.y, cx[i], cy[i]);
        if (d < min_dist) {
            min_dist = d;
            min_idx  = i;
        }
    }
    return min_idx;
}

// =====================================================================
//  增量最近点搜索（局部窗口，O(k)，k = nearest_search_back * 2）
//  在上一帧最近点前后 nearest_search_back 范围内重新找最近点，
//  保证索引只能前进（不回退），以避免车辆回头。
// =====================================================================
int PurePursuitComponent::calcNearestIndexIncremental(
    const std::vector<double>& cx,
    const std::vector<double>& cy) const
{
    const int n     = static_cast<int>(cx.size());
    // 搜索窗口：从 nearest_idx_ 向后回溯，同时向前看
    const int start = std::max(0, nearest_idx_ - cfg_.nearest_search_back);
    const int end   = std::min(n - 1, nearest_idx_ + cfg_.nearest_search_back * 3);

    double min_dist = std::numeric_limits<double>::max();
    int    min_idx  = nearest_idx_;

    for (int i = start; i <= end; ++i) {
        double d = dist(pose_.x, pose_.y, cx[i], cy[i]);
        if (d < min_dist) {
            min_dist = d;
            min_idx  = i;
        }
    }

    // 索引不能回退（防止车辆轻微倒退时重新跟踪已过路点）
    return (min_idx >= nearest_idx_) ? min_idx : nearest_idx_;
}

// =====================================================================
//  曲率 → 目标线速度（线性插值）
//  curvature = 0        → max_velocity
//  curvature = max_curv → min_velocity
// =====================================================================
double PurePursuitComponent::curvatureToVelocity(double curvature) const {
    double ratio = std::clamp(curvature / cfg_.max_curvature, 0.0, 1.0);
    // 线性：直线快、弯道慢
    return cfg_.max_velocity - ratio * (cfg_.max_velocity - cfg_.min_velocity);
}

// =====================================================================
//  alpha 规范化到 (-π, π]，并处理 ±π 边界抖动
// =====================================================================
double PurePursuitComponent::normalizeAlpha(double alpha) {
    // 映射到 (-π, π]
    alpha = std::fmod(alpha + M_PI, 2.0 * M_PI);
    if (alpha < 0.0) alpha += 2.0 * M_PI;
    alpha -= M_PI;

    // 边界抖动保护：若 alpha 非常接近 ±π，轻微偏移避免 sin(alpha) ≈ 0 引起方向反转
    constexpr double kBoundaryEps  = 0.05;   // 判定边界区域的阈值 [rad]
    constexpr double kBoundaryNudge = 0.10;  // 推离边界的偏移量 [rad]
    if (std::abs(std::abs(alpha) - M_PI) < kBoundaryEps) {
        alpha += kBoundaryNudge;
    }

    return alpha;
}

// =====================================================================
//  欧氏距离
// =====================================================================
double PurePursuitComponent::dist(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

// =====================================================================
//  calcPathStartYaw：取最近点起始 N 个路径点的整体朝向
//  用端点连线而非逐点平均，对噪声更鲁棒
// =====================================================================
double PurePursuitComponent::calcPathStartYaw(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    int start_idx) const
{
    const int n   = static_cast<int>(cx.size());
    const int end = std::min(start_idx + cfg_.align_path_points, n - 1);

    if (end <= start_idx) {
        // 路径只有一个点，无法确定方向，返回 0
        return 0.0;
    }

    return std::atan2(cy[end] - cy[start_idx],
                      cx[end] - cx[start_idx]);
}

// =====================================================================
//  computeAlign：对齐阶段原地旋转控制（起始/终点对齐复用）
//  使用 P 控制器，输出 {0.0, w}
//  heading_error 已归一化到 (-π, π]，符号直接决定转向方向：
//    > 0 → 左转（w > 0，逆时针）
//    < 0 → 右转（w < 0，顺时针）
// =====================================================================
std::pair<double, double> PurePursuitComponent::computeAlign(
    double heading_error, double kp, double max_w)
{
    double w = kp * heading_error;
    w = std::clamp(w, -max_w, max_w);
    return {0.0, w};
}

} // namespace pure_pursuit
