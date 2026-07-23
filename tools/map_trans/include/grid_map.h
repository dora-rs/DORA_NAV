#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

// 栅格地图类
class GridMap {
public:
    GridMap();
    ~GridMap();

    // 从点云生成栅格地图
    bool generateFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               double resolution,
                               double x_min, double x_max,
                               double y_min, double y_max);

    // 获取地图数据
    const std::vector<unsigned char>& getData() const { return data_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }

private:
    std::vector<unsigned char> data_;  // 地图数据（行优先存储）
    int width_;                        // 地图宽度（像素）
    int height_;                       // 地图高度（像素）
    double resolution_;                // 分辨率（米/像素）
    double origin_x_;                  // 地图原点X坐标（米）
    double origin_y_;                  // 地图原点Y坐标（米）
};

#endif // GRID_MAP_H
