#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <yaml-cpp/yaml.h>

namespace hdl_graph_slam_dora {

class PrefilteringModule {
public:
    PrefilteringModule();
    void loadConfig(const YAML::Node& config);
    pcl::PointCloud<pcl::PointXYZI>::Ptr process(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud
    );

private:
    // 配置参数
    std::string downsample_method_;
    double downsample_resolution_;
    bool use_distance_filter_;
    double distance_near_thresh_;
    double distance_far_thresh_;
    std::string outlier_removal_method_;
    int statistical_mean_k_;
    double statistical_stddev_;
    double radius_radius_;
    int radius_min_neighbors_;

    // PCL 滤波器
    pcl::VoxelGrid<pcl::PointXYZI> voxelgrid_;
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> approx_voxelgrid_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statistical_filter_;
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_filter_;

    void applyDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void applyDistanceFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void applyOutlierRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
};

} // namespace hdl_graph_slam_dora
