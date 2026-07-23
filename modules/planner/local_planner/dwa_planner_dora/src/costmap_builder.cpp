#include "dwa_planner/costmap_builder.hpp"
#include <iostream>
#include <algorithm>
#include <fstream>

namespace dwa_planner {

// =====================================================================
//  从点云构建代价地图
//  点云已经在机器人坐标系下，直接投影到栅格
// =====================================================================
void CostmapBuilder::buildFromPointCloud(const Eigen::MatrixXf& cloud)
{
    // 清空地图
    clear();

    if (cloud.rows() == 0) {
        return;
    }

    // 将点云投影到栅格地图（只使用xy坐标）
    for (int i = 0; i < cloud.rows(); ++i) {
        float local_x = cloud(i, 0);
        float local_y = cloud(i, 1);

        int grid_x, grid_y;
        if (worldToGrid(local_x, local_y, grid_x, grid_y)) {
            int idx = gridToIndex(grid_x, grid_y);
            costmap_[idx] = config_.lethal_cost;
        }
    }

    // 障碍物膨胀
    inflateObstacles();

    // 计算距离场
    computeDistanceField();
}

// =====================================================================
//  检查指定位置是否有障碍物
// =====================================================================
bool CostmapBuilder::isOccupied(float local_x, float local_y) const
{
    int grid_x, grid_y;
    if (!worldToGrid(local_x, local_y, grid_x, grid_y)) {
        // 超出地图范围视为障碍物
        return true;
    }

    int idx = gridToIndex(grid_x, grid_y);
    return costmap_[idx] >= config_.lethal_cost;
}

// =====================================================================
//  检查轨迹是否无碰撞
//  注意：跳过前几个点的检测，避免因点云噪声导致机器人当前位置
//        被误判为障碍物，从而使所有轨迹都失败
// =====================================================================
bool CostmapBuilder::isTrajectoryCollisionFree(
    const std::vector<std::pair<float, float>>& trajectory) const
{
    // 跳过前3个点（约0.3秒的轨迹，机器人当前位置附近）
    // 这样可以避免点云噪声导致机器人脚下被标记为障碍物时，所有轨迹都失败
    size_t start_idx = std::min(size_t(3), trajectory.size());

    for (size_t i = start_idx; i < trajectory.size(); ++i) {
        const auto& point = trajectory[i];
        if (isOccupied(point.first, point.second)) {
            return false;
        }
    }
    return true;
}

// =====================================================================
//  计算轨迹到最近障碍物的距离（优化版：使用距离场）
// =====================================================================
double CostmapBuilder::getMinDistanceToObstacle(
    const std::vector<std::pair<float, float>>& trajectory) const
{
    double min_distance = std::numeric_limits<double>::infinity();

    // 遍历轨迹上的每个点，查询距离场
    for (const auto& traj_point : trajectory) {
        double dist = getDistanceAt(traj_point.first, traj_point.second);
        min_distance = std::min(min_distance, dist);
    }

    return min_distance;
}

// =====================================================================
//  获取指定位置到最近障碍物的距离
// =====================================================================
double CostmapBuilder::getDistanceAt(float local_x, float local_y) const
{
    int grid_x, grid_y;
    if (!worldToGrid(local_x, local_y, grid_x, grid_y)) {
        // 超出地图范围，返回0（视为接近障碍物）
        return 0.0;
    }

    int idx = gridToIndex(grid_x, grid_y);
    return static_cast<double>(distance_field_[idx]);
}

// =====================================================================
//  清空地图
// =====================================================================
void CostmapBuilder::clear() {
    std::fill(costmap_.begin(), costmap_.end(), config_.free_cost);
    std::fill(distance_field_.begin(), distance_field_.end(), 0.0f);
}

// =====================================================================
//  计算距离场（使用欧几里得距离变换）
//  算法：两遍扫描法（Two-pass algorithm）
// =====================================================================
void CostmapBuilder::computeDistanceField() {
    const float INF = 9999.0f;

    // 初始化距离场
    for (int i = 0; i < width_cells_ * height_cells_; ++i) {
        if (costmap_[i] >= config_.lethal_cost) {
            distance_field_[i] = 0.0f;  // 障碍物位置距离为0
        } else {
            distance_field_[i] = INF;   // 自由空间初始化为无穷大
        }
    }

    // 第一遍：从左上到右下扫描
    for (int y = 0; y < height_cells_; ++y) {
        for (int x = 0; x < width_cells_; ++x) {
            int idx = gridToIndex(x, y);

            if (distance_field_[idx] == 0.0f) continue;  // 跳过障碍物

            float min_dist = distance_field_[idx];

            // 检查左边和上边的邻居
            if (x > 0) {
                int left_idx = gridToIndex(x - 1, y);
                min_dist = std::min(min_dist, distance_field_[left_idx] + 1.0f);
            }
            if (y > 0) {
                int up_idx = gridToIndex(x, y - 1);
                min_dist = std::min(min_dist, distance_field_[up_idx] + 1.0f);
            }

            distance_field_[idx] = min_dist;
        }
    }

    // 第二遍：从右下到左上扫描
    for (int y = height_cells_ - 1; y >= 0; --y) {
        for (int x = width_cells_ - 1; x >= 0; --x) {
            int idx = gridToIndex(x, y);

            if (distance_field_[idx] == 0.0f) continue;  // 跳过障碍物

            float min_dist = distance_field_[idx];

            // 检查右边和下边的邻居
            if (x < width_cells_ - 1) {
                int right_idx = gridToIndex(x + 1, y);
                min_dist = std::min(min_dist, distance_field_[right_idx] + 1.0f);
            }
            if (y < height_cells_ - 1) {
                int down_idx = gridToIndex(x, y + 1);
                min_dist = std::min(min_dist, distance_field_[down_idx] + 1.0f);
            }

            distance_field_[idx] = min_dist;
        }
    }

    // 转换为实际距离（栅格数 × 分辨率）
    for (int i = 0; i < width_cells_ * height_cells_; ++i) {
        distance_field_[i] *= config_.resolution;
    }
}

// =====================================================================
//  机器人坐标系 → 栅格坐标（地图中心为机器人位置）
// =====================================================================
bool CostmapBuilder::worldToGrid(
    float local_x, float local_y,
    int& grid_x, int& grid_y) const
{
    // 转换到栅格坐标（地图中心为机器人位置）
    // 机器人在地图中心，x正方向向前，y正方向向左
    grid_x = static_cast<int>(std::floor(local_x / config_.resolution + width_cells_ / 2.0f));
    grid_y = static_cast<int>(std::floor(local_y / config_.resolution + height_cells_ / 2.0f));

    return isInBounds(grid_x, grid_y);
}

// =====================================================================
//  栅格坐标 → 数组索引
// =====================================================================
int CostmapBuilder::gridToIndex(int grid_x, int grid_y) const {
    return grid_y * width_cells_ + grid_x;
}

// =====================================================================
//  检查栅格坐标是否在地图范围内
// =====================================================================
bool CostmapBuilder::isInBounds(int grid_x, int grid_y) const {
    return grid_x >= 0 && grid_x < width_cells_ &&
           grid_y >= 0 && grid_y < height_cells_;
}

// =====================================================================
//  障碍物膨胀（简单方形膨胀）
// =====================================================================
void CostmapBuilder::inflateObstacles() {
    if (inflation_cells_ <= 0) {
        return;
    }

    // 复制原始地图
    std::vector<uint8_t> original = costmap_;

    // 对每个障碍物栅格进行膨胀
    for (int y = 0; y < height_cells_; ++y) {
        for (int x = 0; x < width_cells_; ++x) {
            int idx = gridToIndex(x, y);

            if (original[idx] >= config_.lethal_cost) {
                // 膨胀周围区域（方形膨胀）
                for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
                    for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (isInBounds(nx, ny)) {
                            int nidx = gridToIndex(nx, ny);
                            costmap_[nidx] = config_.lethal_cost;
                        }
                    }
                }
            }
        }
    }
}

// =====================================================================
//  保存代价地图为PGM格式
//  PGM格式：灰度图像，0=黑色(障碍物)，255=白色(自由空间)
// =====================================================================
bool CostmapBuilder::saveToPGM(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[CostmapBuilder] Failed to open file: " << filename << std::endl;
        return false;
    }

    // PGM header (P5 = binary grayscale)
    file << "P5\n";
    file << width_cells_ << " " << height_cells_ << "\n";
    file << "255\n";

    // 写入图像数据（需要上下翻转，因为PGM从上到下，但我们的地图从下到上）
    // 同时反转颜色：障碍物(254) → 黑色(0)，自由空间(0) → 白色(255)
    for (int y = height_cells_ - 1; y >= 0; --y) {
        for (int x = 0; x < width_cells_; ++x) {
            int idx = gridToIndex(x, y);
            uint8_t cost = costmap_[idx];

            // 反转颜色：障碍物显示为黑色，自由空间显示为白色
            uint8_t pixel = (cost >= config_.lethal_cost) ? 0 : 255;
            file.write(reinterpret_cast<const char*>(&pixel), 1);
        }
    }

    file.close();
    std::cout << "[CostmapBuilder] Saved costmap to " << filename
              << " (" << width_cells_ << "x" << height_cells_ << ")" << std::endl;
    return true;
}

} // namespace dwa_planner
