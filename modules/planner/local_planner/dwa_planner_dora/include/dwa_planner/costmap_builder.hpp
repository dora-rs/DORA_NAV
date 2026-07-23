#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "types.hpp"

namespace dwa_planner {

// =====================================================================
//  局部代价地图配置参数
// =====================================================================
struct CostmapConfig {
    // 地图尺寸（以机器人为中心）
    float width  = 6.0f;   // 地图宽度 [m]（前后各3m）
    float height = 6.0f;   // 地图高度 [m]（左右各3m）
    float resolution = 0.1f;  // 栅格分辨率 [m]（平衡精度与计算量）

    // 障碍物膨胀
    float robot_radius = 0.15f;  // 机器人半径 [m]（用于障碍物膨胀）

    // 代价值
    uint8_t lethal_cost = 254;  // 致命障碍物代价
    uint8_t free_cost   = 0;    // 自由空间代价
};

// =====================================================================
//  局部代价地图构建器
//  功能：将点云投影到2D栅格地图，提供碰撞检测接口
//  注意：地图以机器人为中心，点云已经在机器人坐标系下
// =====================================================================
class CostmapBuilder {
public:
    explicit CostmapBuilder(const CostmapConfig& config = CostmapConfig{})
        : config_(config) {
        // 计算栅格数量
        width_cells_  = static_cast<int>(std::ceil(config_.width / config_.resolution));
        height_cells_ = static_cast<int>(std::ceil(config_.height / config_.resolution));

        // 计算膨胀半径（栅格数）
        inflation_cells_ = static_cast<int>(std::ceil(config_.robot_radius / config_.resolution));

        // 初始化代价地图和距离场
        costmap_.resize(height_cells_ * width_cells_, config_.free_cost);
        distance_field_.resize(height_cells_ * width_cells_, 0.0f);
    }

    // 从点云构建代价地图（点云已在机器人坐标系下）
    void buildFromPointCloud(const Eigen::MatrixXf& cloud);

    // 检查指定位置是否有障碍物（机器人坐标系）
    bool isOccupied(float local_x, float local_y) const;

    // 检查轨迹是否碰撞（用于DWA轨迹评估）
    bool isTrajectoryCollisionFree(const std::vector<std::pair<float, float>>& trajectory) const;

    // 计算轨迹到最近障碍物的距离（用于避障代价计算）
    double getMinDistanceToObstacle(const std::vector<std::pair<float, float>>& trajectory) const;

    // 获取指定位置到最近障碍物的距离（使用距离场）
    double getDistanceAt(float local_x, float local_y) const;

    // 清空地图
    void clear();

    // 获取地图信息（供调试使用）
    int getWidthCells() const { return width_cells_; }
    int getHeightCells() const { return height_cells_; }
    const std::vector<uint8_t>& getCostmap() const { return costmap_; }

    // 保存代价地图为PGM格式（用于可视化验证）
    bool saveToPGM(const std::string& filename) const;

private:
    // 机器人坐标系 → 栅格坐标（地图中心为机器人位置）
    bool worldToGrid(float local_x, float local_y, int& grid_x, int& grid_y) const;

    // 栅格坐标 → 数组索引
    int gridToIndex(int grid_x, int grid_y) const;

    // 检查栅格坐标是否在地图范围内
    bool isInBounds(int grid_x, int grid_y) const;

    // 障碍物膨胀
    void inflateObstacles();

    // 计算距离场（距离变换）
    void computeDistanceField();

    CostmapConfig config_;

    int width_cells_;      // 地图宽度（栅格数）
    int height_cells_;     // 地图高度（栅格数）
    int inflation_cells_;  // 膨胀半径（栅格数）

    std::vector<uint8_t> costmap_;  // 代价地图（行优先存储）
    std::vector<float> distance_field_;  // 距离场（每个栅格到最近障碍物的距离，单位：米）
};

} // namespace dwa_planner
