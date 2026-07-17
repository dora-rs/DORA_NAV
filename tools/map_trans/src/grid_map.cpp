#include "grid_map.h"
#include <iostream>
#include <cmath>

// 栅格值定义
const unsigned char OCCUPIED = 0;    // 占据（黑色）
const unsigned char FREE = 255;      // 空闲（白色）
const unsigned char UNKNOWN = 128;   // 未知（灰色）

GridMap::GridMap()
    : width_(0)
    , height_(0)
    , resolution_(0.0)
    , origin_x_(0.0)
    , origin_y_(0.0)
{
}

GridMap::~GridMap() {
}

bool GridMap::generateFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    double resolution,
                                    double x_min, double x_max,
                                    double y_min, double y_max) {
    if (cloud->points.empty()) {
        std::cerr << "错误: 点云为空，无法生成地图" << std::endl;
        return false;
    }

    if (resolution <= 0.0) {
        std::cerr << "错误: 分辨率必须大于0" << std::endl;
        return false;
    }

    resolution_ = resolution;
    origin_x_ = x_min;
    origin_y_ = y_min;

    // 计算地图尺寸
    width_ = static_cast<int>(std::ceil((x_max - x_min) / resolution_));
    height_ = static_cast<int>(std::ceil((y_max - y_min) / resolution_));

    std::cout << "生成栅格地图:" << std::endl;
    std::cout << "  尺寸: " << width_ << " x " << height_ << " 像素" << std::endl;
    std::cout << "  分辨率: " << resolution_ << " 米/像素" << std::endl;
    std::cout << "  原点: (" << origin_x_ << ", " << origin_y_ << ") 米" << std::endl;

    // 初始化地图数据（所有栅格初始化为未知）
    data_.resize(width_ * height_, UNKNOWN);

    // 将点云投影到栅格地图
    int occupied_count = 0;
    for (const auto& point : cloud->points) {
        // 计算栅格坐标
        int grid_x = static_cast<int>((point.x - origin_x_) / resolution_);
        int grid_y = static_cast<int>((point.y - origin_y_) / resolution_);

        // 检查是否在地图范围内
        if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
            // 计算数组索引（行优先存储）
            int index = grid_y * width_ + grid_x;

            // 标记为占据
            if (data_[index] != OCCUPIED) {
                data_[index] = OCCUPIED;
                occupied_count++;
            }
        }
    }

    std::cout << "地图生成完成，占据栅格数: " << occupied_count << std::endl;

    return true;
}
