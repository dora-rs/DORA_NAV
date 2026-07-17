#include "hdl_graph_slam_dora/map_saver.h"
#include "hdl_graph_slam_dora/scan_matching_odometry_module.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

namespace hdl_graph_slam_dora {

MapSaver::MapSaver()
    : destination_path_("../maps/hdl_slam_map.pcd")
    , map_cloud_resolution_(0.05)
{
}

void MapSaver::loadConfig(const YAML::Node& config) {
    destination_path_ = config["destination"].as<std::string>("../maps/hdl_slam_map.pcd");
    map_cloud_resolution_ = config["map_cloud_resolution"].as<double>(0.05);

    std::cout << "[MapSaver] Loaded config: destination=" << destination_path_
              << ", resolution=" << map_cloud_resolution_ << "m" << std::endl;
}

bool MapSaver::saveMap(const std::vector<std::shared_ptr<KeyFrame>>& keyframes) {
    if (keyframes.empty()) {
        std::cerr << "[MapSaver] No keyframes to save!" << std::endl;
        return false;
    }

    std::cout << "[MapSaver] Generating map from " << keyframes.size() << " keyframes..." << std::endl;

    // 累积所有关键帧点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (const auto& keyframe : keyframes) {
        if (!keyframe->cloud || keyframe->cloud->empty()) {
            continue;
        }

        // 将点云转换到世界坐标系
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*keyframe->cloud, *transformed_cloud, keyframe->pose.matrix().cast<float>());

        *map_cloud += *transformed_cloud;
    }

    std::cout << "[MapSaver] Map generated | Total points: " << map_cloud->size() << std::endl;

    // 降采样
    if (map_cloud_resolution_ > 0.0) {
        std::cout << "[MapSaver] Downsampling to resolution " << map_cloud_resolution_ << "m..." << std::endl;
        pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
        voxelgrid.setLeafSize(map_cloud_resolution_, map_cloud_resolution_, map_cloud_resolution_);
        voxelgrid.setInputCloud(map_cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        voxelgrid.filter(*filtered);
        map_cloud = filtered;

        std::cout << "[MapSaver] Map downsampled | Points: " << map_cloud->size() << std::endl;
    }

    // 保存为 PCD 文件
    std::cout << "[MapSaver] Saving to: " << destination_path_ << std::endl;
    if (pcl::io::savePCDFileBinary(destination_path_, *map_cloud) == -1) {
        std::cerr << "[MapSaver] Failed to save PCD file!" << std::endl;
        return false;
    }

    std::cout << "[MapSaver] Map saved successfully!" << std::endl;
    return true;
}

} // namespace hdl_graph_slam_dora
