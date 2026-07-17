//
// Created by xiang on 25-6-23.
//

#ifndef LIGHTNING_G2P5_MAP_H
#define LIGHTNING_G2P5_MAP_H

#include "common/eigen_types.h"
#include "common/std_types.h"
#include "core/g2p5/g2p5_subgrid.h"

#include <bitset>
// #include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>

namespace lightning::g2p5 {

/**
 * g2p5自定义栅格地图数据
 * 主要用于对外的绘制，并没有匹配部分
 * 数据存储在grids_中，是一个二维数组，内部还有4层subgrids
 * x为行指针，y为列指针，y优先增长
 *
 * 可以通过ToCV 或者 ToROS 转换成OpenCV格式或者ROS格式进行显示和存储
 */
class G2P5Map {
   public:
    struct Options {
        float resolution_ = 0.05;      /// 子网格的最高分辨率
        float occupancy_ratio_ = 0.3;  // 占用比例
    };

    G2P5Map(Options options) : options_(options) {
        grid_reso_ = options_.resolution_ * sub_grid_width_;
        grids_ = nullptr;
    }

    ~G2P5Map();

    /// 子网格的大小
    static inline constexpr int SUB_GRID_SIZE = SubGrid::SUB_GRID_SIZE;

    /// 由自身内容创建一个深拷贝
    std::shared_ptr<G2P5Map> MakeDeepCopy();

    /// 转换至ros occupancy grid
    // nav_msgs::msg::OccupancyGrid ToROS();

    /// 转换至opencv::Mat
    cv::Mat ToCV();

    /// 清空内部数据
    void ReleaseResources();

    bool Init(const float &temp_min_x, const float &temp_min_y, const float &temp_max_x, const float &temp_max_y);

    bool Resize(const float &temp_min_x, const float &temp_min_y, const float &temp_max_x, const float &temp_max_y);

    void SetHitPoint(const float &px, const float &py, const bool &if_hit, float height);

    /**
     * 白色区域的直线填充算法
     * 除了2D填充以外，还需要给出雷达高度和目标位置高度
     */
    void SetMissPoint(const float &point_x, const float &point_y, const float &laser_origin_x,
                      const float &laser_origin_y, float height, float lidar_height);

    bool IsObstacle(const Vec2i &point) {
        // 判断点是否属于障碍物区域，可以是通过查找 grid 中是否已经有墙体或物体数据
        int xi = point.x() >> SUB_GRID_SIZE;
        int yi = point.y() >> SUB_GRID_SIZE;
        if (xi < 0 || xi >= grid_size_x_ || yi < 0 || yi >= grid_size_y_) {
            return false;
        }
        return true;
    }

    void UpdateCell(const Vec2i &point_index, const bool &if_hit, float height);

    bool GetDataIndex(const float x, const float y, int &x_index, int &y_index);

    bool IsEmpty() { return (grids_ == nullptr); }

    inline void GetMinAndMax(float &min_x, float &min_y, float &max_x, float &max_y) {
        min_x = min_x_;
        min_y = min_y_;
        max_x = max_x_;
        max_y = max_y_;
    }
    inline void SetMinAndMax(float &min_x, float &min_y, float &max_x, float &max_y) {
        min_x_ = min_x;
        min_y_ = min_y;
        max_x_ = max_x;
        max_y_ = max_y;
    }

    float GetGridResolution() const { return options_.resolution_; }

   private:
    inline int MapIdx(int sx, int x, int y) { return (sx) * (y) + (x); }

   private:
    float grid_reso_ = 0.0;  /// subgrid的栅格分辨率
    float min_x_ = 0, min_y_ = 0, max_x_ = 0, max_y_ = 0;
    int grid_size_x_ = 0, grid_size_y_ = 0;

    inline static const int sub_grid_width_ = (1 << SUB_GRID_SIZE);

    Options options_;

    SubGrid **grids_ = nullptr;  // 实际存储数据的位置
};

typedef std::shared_ptr<G2P5Map> G2P5MapPtr;

}  // namespace lightning::g2p5

#endif  // LIGHTNING_G2P5_MAP_H
