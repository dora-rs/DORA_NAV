#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "types.hpp"
#include "costmap_builder.hpp"

namespace dwa_planner {

// =====================================================================
//  DWA算法配置参数（简化版用于测试）
// =====================================================================
struct DWAConfig {
    // 速度限制
    double fixed_v = 0.2;    // 固定线速度 [m/s]（简化测试）
    double max_w = 2.0;      // 最大角速度 [rad/s]（增大以提高机动性）

    // 加速度限制
    double max_accel_w = 5.0;   // 最大角加速度 [rad/s²]（扩大动态窗口到±0.5 rad/s）

    // 采样参数
    int w_samples = 20;          // 角速度采样数

    // 预测参数
    double predict_time = 4.0;   // 轨迹预测时间 [s]（预测距离0.4m）
    double dt = 0.05;            // 预测时间步长 [s]（减小步长避免过度偏转）

    // 评分函数权重（得分越高越好）
    double weight_obstacle = 2.0;  // 避障得分权重（提高优先级）
    double weight_heading = 0.6;   // 朝向得分权重

    // 速度平滑参数
    double velocity_smooth_alpha = 1.0;  // 平滑系数 [0-1]，提高响应速度
};

// =====================================================================
//  DWA局部规划器核心组件
//  功能：在跟随全局路径的前提下实现动态避障
// =====================================================================
class DWAPlannerComponent {
public:
    explicit DWAPlannerComponent(const DWAConfig& config = DWAConfig{})
        : config_(config) {}

    // 主计算接口
    // 输入：当前机器人状态、局部坐标系路径、代价地图
    // 输出：最优速度指令 (v, w)
    std::pair<double, double> compute(
        const RobotState& current_state,
        const std::vector<double>& local_path_x,
        const std::vector<double>& local_path_y,
        const CostmapBuilder& costmap
    );

    // 获取所有预测轨迹（用于可视化）
    const std::vector<std::vector<std::pair<float, float>>>&
        getAllTrajectories() const {
        return all_trajectories_;
    }

private:
    // 1. 计算角速度动态窗口
    // 考虑机器人物理限制、加速度限制
    void calculateDynamicWindow(
        double current_w,
        double& w_min, double& w_max
    );

    // 2. 预测轨迹（差分驱动运动学模型）
    // 输入：当前状态、速度指令(v, w)、可选的终点距离限制
    // 输出：预测轨迹点序列
    std::vector<std::pair<float, float>> predictTrajectory(
        const RobotState& state, double v, double w,
        double goal_distance = -1.0  // 负值表示不限制
    );

    // 3. 评估轨迹总得分（得分越高越好）
    double evaluateTrajectory(
        const std::vector<std::pair<float, float>>& trajectory,
        const std::vector<double>& path_x,
        const std::vector<double>& path_y,
        const CostmapBuilder& costmap
    );

    // 3.1 避障得分（距离障碍物越远得分越高，碰撞得分为负无穷）
    double calculateObstacleScore(
        const std::vector<std::pair<float, float>>& trajectory,
        const CostmapBuilder& costmap
    );

    // 3.2 朝向得分（轨迹终点朝向与局部目标方向的夹角越小得分越高）
    double calculateHeadingScore(
        const std::vector<std::pair<float, float>>& trajectory,
        const std::vector<double>& path_x,
        const std::vector<double>& path_y
    );

    // 4. 速度平滑（指数移动平均）
    void smoothVelocity(double& w);

    // 5. 可视化（用于调试）
    // 绘制局部代价地图和所有预测轨迹
    void visualize(const CostmapBuilder& costmap);

    // 辅助函数：计算两点距离
    double distance(double x1, double y1, double x2, double y2) const {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    // 配置参数
    DWAConfig config_;

    // 速度平滑历史
    double last_v_ = 0.0;
    double last_w_ = 0.0;

    // 所有预测轨迹（用于可视化）
    std::vector<std::vector<std::pair<float, float>>> all_trajectories_;

    // 可视化配置
    bool enable_visualization_ = false;  // 是否启用可视化
    int viz_scale_ = 100;  // 可视化缩放因子（像素/米）
};

} // namespace dwa_planner
