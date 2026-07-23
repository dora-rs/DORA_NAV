#include "config_loader.h"
#include "pcd_reader.h"
#include "grid_map.h"
#include "pgm_writer.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "  PCD点云地图转PGM栅格地图转换程序" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // 1. 加载配置文件
    Config config;
    std::string config_file = "../config/config.json";

    if (argc > 1) {
        config_file = argv[1];
    }

    if (!ConfigLoader::loadConfig(config_file, config)) {
        std::cout << "使用默认配置继续..." << std::endl;
    }

    // 验证配置
    if (!ConfigLoader::validateConfig(config)) {
        std::cerr << "配置验证失败，程序退出" << std::endl;
        return -1;
    }

    // 打印配置信息
    ConfigLoader::printConfig(config);

    // 2. 读取PCD文件
    std::cout << "========== 步骤1: 读取PCD文件 ==========" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (!PCDReader::loadPCD(config.pcd_file, cloud)) {
        std::cerr << "读取PCD文件失败，程序退出" << std::endl;
        return -1;
    }

    if (cloud->points.empty()) {
        std::cerr << "错误: 点云为空，程序退出" << std::endl;
        return -1;
    }

    // 3. 点云预处理
    std::cout << "\n========== 步骤2: 点云预处理 ==========" << std::endl;

    // 降采样（可选）
    if (config.enable_downsample) {
        PCDReader::downsample(cloud, config.voxel_size);
    }

    // 去除离群点（可选）
    if (config.remove_outliers) {
        PCDReader::removeOutliers(cloud, config.outlier_radius, config.outlier_min_neighbors);
    }

    // 计算点云边界
    double x_min, x_max, y_min, y_max, z_min, z_max;
    PCDReader::computeBounds(cloud, x_min, x_max, y_min, y_max, z_min, z_max);

    // 高度过滤
    PCDReader::filterByHeight(cloud, config.min_height, config.max_height);

    if (cloud->points.empty()) {
        std::cerr << "错误: 高度过滤后点云为空，程序退出" << std::endl;
        return -1;
    }

    // 重新计算边界（高度过滤后）
    PCDReader::computeBounds(cloud, x_min, x_max, y_min, y_max, z_min, z_max);

    // 4. 生成栅格地图
    std::cout << "\n========== 步骤3: 生成栅格地图 ==========" << std::endl;
    GridMap grid_map;

    if (!grid_map.generateFromPointCloud(cloud, config.resolution,
                                        x_min, x_max, y_min, y_max)) {
        std::cerr << "生成栅格地图失败，程序退出" << std::endl;
        return -1;
    }

    // 5. 保存PGM文件
    std::cout << "\n========== 步骤4: 保存地图文件 ==========" << std::endl;

    if (!PGMWriter::savePGM(grid_map, config.pgm_file)) {
        std::cerr << "保存PGM文件失败" << std::endl;
        return -1;
    }

    // 6. 保存YAML元数据文件
    if (!PGMWriter::saveYAML(grid_map, config.pgm_file, config.yaml_file)) {
        std::cerr << "保存YAML文件失败" << std::endl;
        return -1;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "  转换完成！" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
