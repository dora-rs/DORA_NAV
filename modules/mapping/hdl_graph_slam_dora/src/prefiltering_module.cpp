#include "hdl_graph_slam_dora/prefiltering_module.h"
#include <iostream>

namespace hdl_graph_slam_dora {

PrefilteringModule::PrefilteringModule()
    : downsample_method_("VOXELGRID")
    , downsample_resolution_(0.1)
    , use_distance_filter_(false)
    , distance_near_thresh_(1.0)
    , distance_far_thresh_(100.0)
    , outlier_removal_method_("NONE")
    , statistical_mean_k_(30)
    , statistical_stddev_(1.2)
    , radius_radius_(0.5)
    , radius_min_neighbors_(2)
{
}

void PrefilteringModule::loadConfig(const YAML::Node& config) {
    downsample_method_ = config["downsample_method"].as<std::string>("VOXELGRID");
    downsample_resolution_ = config["downsample_resolution"].as<double>(0.1);
    use_distance_filter_ = config["use_distance_filter"].as<bool>(false);
    distance_near_thresh_ = config["distance_near_thresh"].as<double>(1.0);
    distance_far_thresh_ = config["distance_far_thresh"].as<double>(100.0);
    outlier_removal_method_ = config["outlier_removal_method"].as<std::string>("NONE");

    if (outlier_removal_method_ == "STATISTICAL") {
        statistical_mean_k_ = config["statistical_mean_k"].as<int>(30);
        statistical_stddev_ = config["statistical_stddev"].as<double>(1.2);
    } else if (outlier_removal_method_ == "RADIUS") {
        radius_radius_ = config["radius_radius"].as<double>(0.5);
        radius_min_neighbors_ = config["radius_min_neighbors"].as<int>(2);
    }

    std::cout << "[Prefiltering] Loaded config: method=" << downsample_method_
              << ", resolution=" << downsample_resolution_ << std::endl;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PrefilteringModule::process(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud
) {
    if (!input_cloud || input_cloud->empty()) {
        return input_cloud;
    }

    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*input_cloud);

    applyDownsample(cloud);

    if (use_distance_filter_) {
        applyDistanceFilter(cloud);
    }

    if (outlier_removal_method_ != "NONE") {
        applyOutlierRemoval(cloud);
    }

    return cloud;
}

void PrefilteringModule::applyDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (downsample_method_ == "NONE" || cloud->empty()) {
        return;
    }

    if (downsample_method_ == "VOXELGRID") {
        voxelgrid_.setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
        voxelgrid_.setInputCloud(cloud);

        auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        voxelgrid_.filter(*filtered);
        cloud = filtered;

    } else if (downsample_method_ == "APPROX_VOXELGRID") {
        approx_voxelgrid_.setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
        approx_voxelgrid_.setInputCloud(cloud);

        auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        approx_voxelgrid_.filter(*filtered);
        cloud = filtered;
    }
}

void PrefilteringModule::applyDistanceFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (cloud->empty()) {
        return;
    }

    auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    filtered->reserve(cloud->size());

    double near_sq = distance_near_thresh_ * distance_near_thresh_;
    double far_sq = distance_far_thresh_ * distance_far_thresh_;

    for (const auto& p : cloud->points) {
        double dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
        if (dist_sq >= near_sq && dist_sq <= far_sq) {
            filtered->points.push_back(p);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = false;
    cloud = filtered;
}

void PrefilteringModule::applyOutlierRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (cloud->empty()) {
        return;
    }

    if (outlier_removal_method_ == "STATISTICAL") {
        statistical_filter_.setMeanK(statistical_mean_k_);
        statistical_filter_.setStddevMulThresh(statistical_stddev_);
        statistical_filter_.setInputCloud(cloud);

        auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        statistical_filter_.filter(*filtered);
        cloud = filtered;

    } else if (outlier_removal_method_ == "RADIUS") {
        radius_filter_.setRadiusSearch(radius_radius_);
        radius_filter_.setMinNeighborsInRadius(radius_min_neighbors_);
        radius_filter_.setInputCloud(cloud);

        auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        radius_filter_.filter(*filtered);
        cloud = filtered;
    }
}

} // namespace hdl_graph_slam_dora
