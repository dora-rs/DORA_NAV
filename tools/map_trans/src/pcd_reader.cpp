#include "pcd_reader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <limits>

bool PCDReader::loadPCD(const std::string& filename,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        std::cerr << "错误: 无法读取PCD文件: " << filename << std::endl;
        return false;
    }

    std::cout << "成功加载PCD文件: " << filename << std::endl;
    std::cout << "点云包含 " << cloud->points.size() << " 个点" << std::endl;

    return true;
}

void PCDReader::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           double voxel_size) {
    std::cout << "执行降采样，体素大小: " << voxel_size << " 米" << std::endl;

    size_t original_size = cloud->points.size();

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*filtered_cloud);

    *cloud = *filtered_cloud;

    std::cout << "降采样完成: " << original_size << " -> "
              << cloud->points.size() << " 个点" << std::endl;
}

void PCDReader::removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               double radius,
                               int min_neighbors) {
    std::cout << "去除离群点，搜索半径: " << radius
              << " 米，最小邻居数: " << min_neighbors << std::endl;

    size_t original_size = cloud->points.size();

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(cloud);
    outlier_filter.setRadiusSearch(radius);
    outlier_filter.setMinNeighborsInRadius(min_neighbors);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    outlier_filter.filter(*filtered_cloud);

    *cloud = *filtered_cloud;

    std::cout << "离群点去除完成: " << original_size << " -> "
              << cloud->points.size() << " 个点" << std::endl;
}

void PCDReader::filterByHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               double min_height,
                               double max_height) {
    std::cout << "高度过滤，范围: [" << min_height << ", "
              << max_height << "] 米" << std::endl;

    size_t original_size = cloud->points.size();

    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud(cloud);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(min_height, max_height);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*filtered_cloud);

    *cloud = *filtered_cloud;

    std::cout << "高度过滤完成: " << original_size << " -> "
              << cloud->points.size() << " 个点" << std::endl;
}

void PCDReader::computeBounds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             double& x_min, double& x_max,
                             double& y_min, double& y_max,
                             double& z_min, double& z_max) {
    if (cloud->points.empty()) {
        std::cerr << "错误: 点云为空，无法计算边界" << std::endl;
        return;
    }

    x_min = y_min = z_min = std::numeric_limits<double>::max();
    x_max = y_max = z_max = std::numeric_limits<double>::lowest();

    for (const auto& point : cloud->points) {
        if (point.x < x_min) x_min = point.x;
        if (point.x > x_max) x_max = point.x;
        if (point.y < y_min) y_min = point.y;
        if (point.y > y_max) y_max = point.y;
        if (point.z < z_min) z_min = point.z;
        if (point.z > z_max) z_max = point.z;
    }

    std::cout << "点云边界:" << std::endl;
    std::cout << "  X: [" << x_min << ", " << x_max << "] 米" << std::endl;
    std::cout << "  Y: [" << y_min << ", " << y_max << "] 米" << std::endl;
    std::cout << "  Z: [" << z_min << ", " << z_max << "] 米" << std::endl;
}
