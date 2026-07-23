#ifndef PCD_READER_H
#define PCD_READER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

// PCD读取器类
class PCDReader {
public:
    // 读取PCD文件
    static bool loadPCD(const std::string& filename,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 点云预处理：降采样
    static void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          double voxel_size);

    // 点云预处理：去除离群点
    static void removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                              double radius,
                              int min_neighbors);

    // 高度过滤
    static void filterByHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                              double min_height,
                              double max_height);

    // 计算点云边界
    static void computeBounds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             double& x_min, double& x_max,
                             double& y_min, double& y_max,
                             double& z_min, double& z_max);
};

#endif // PCD_READER_H
