#include "config_loader.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

// 默认配置
Config::Config()
    : pcd_file("../data/input/map.pcd")
    , pgm_file("../data/output/map.pgm")
    , yaml_file("../data/output/map.yaml")
    , resolution(0.05)
    , min_height(0.1)
    , max_height(2.0)
    , enable_downsample(false)
    , voxel_size(0.05)
    , remove_outliers(true)
    , outlier_radius(0.5)
    , outlier_min_neighbors(5)
{
}

bool ConfigLoader::loadConfig(const std::string& config_file, Config& config) {
    try {
        // 读取JSON文件
        std::ifstream file(config_file);
        if (!file.is_open()) {
            std::cerr << "错误: 无法打开配置文件: " << config_file << std::endl;
            std::cerr << "将使用默认配置" << std::endl;
            return false;
        }

        json j;
        file >> j;
        file.close();

        // 解析输入配置
        if (j.contains("input") && j["input"].contains("pcd_file")) {
            config.pcd_file = j["input"]["pcd_file"].get<std::string>();
        }

        // 解析输出配置
        if (j.contains("output")) {
            if (j["output"].contains("pgm_file")) {
                config.pgm_file = j["output"]["pgm_file"].get<std::string>();
            }
            if (j["output"].contains("yaml_file")) {
                config.yaml_file = j["output"]["yaml_file"].get<std::string>();
            }
        }

        // 解析地图参数
        if (j.contains("map_parameters")) {
            auto& map_params = j["map_parameters"];
            if (map_params.contains("resolution")) {
                config.resolution = map_params["resolution"].get<double>();
            }
            if (map_params.contains("min_height")) {
                config.min_height = map_params["min_height"].get<double>();
            }
            if (map_params.contains("max_height")) {
                config.max_height = map_params["max_height"].get<double>();
            }
        }

        // 解析处理参数
        if (j.contains("processing")) {
            auto& proc_params = j["processing"];
            if (proc_params.contains("enable_downsample")) {
                config.enable_downsample = proc_params["enable_downsample"].get<bool>();
            }
            if (proc_params.contains("voxel_size")) {
                config.voxel_size = proc_params["voxel_size"].get<double>();
            }
            if (proc_params.contains("remove_outliers")) {
                config.remove_outliers = proc_params["remove_outliers"].get<bool>();
            }
            if (proc_params.contains("outlier_radius")) {
                config.outlier_radius = proc_params["outlier_radius"].get<double>();
            }
            if (proc_params.contains("outlier_min_neighbors")) {
                config.outlier_min_neighbors = proc_params["outlier_min_neighbors"].get<int>();
            }
        }

        std::cout << "成功加载配置文件: " << config_file << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "错误: 解析配置文件失败: " << e.what() << std::endl;
        std::cerr << "将使用默认配置" << std::endl;
        return false;
    }
}

bool ConfigLoader::validateConfig(const Config& config) {
    bool valid = true;

    // 验证分辨率
    if (config.resolution <= 0.0 || config.resolution > 1.0) {
        std::cerr << "错误: 分辨率必须在 (0, 1] 范围内" << std::endl;
        valid = false;
    }

    // 验证高度范围
    if (config.min_height >= config.max_height) {
        std::cerr << "错误: min_height 必须小于 max_height" << std::endl;
        valid = false;
    }

    // 验证降采样参数
    if (config.enable_downsample && config.voxel_size <= 0.0) {
        std::cerr << "错误: voxel_size 必须大于 0" << std::endl;
        valid = false;
    }

    // 验证离群点去除参数
    if (config.remove_outliers) {
        if (config.outlier_radius <= 0.0) {
            std::cerr << "错误: outlier_radius 必须大于 0" << std::endl;
            valid = false;
        }
        if (config.outlier_min_neighbors <= 0) {
            std::cerr << "错误: outlier_min_neighbors 必须大于 0" << std::endl;
            valid = false;
        }
    }

    return valid;
}

void ConfigLoader::printConfig(const Config& config) {
    std::cout << "\n========== 配置信息 ==========" << std::endl;
    std::cout << "输入PCD文件: " << config.pcd_file << std::endl;
    std::cout << "输出PGM文件: " << config.pgm_file << std::endl;
    std::cout << "输出YAML文件: " << config.yaml_file << std::endl;
    std::cout << "地图分辨率: " << config.resolution << " 米/像素" << std::endl;
    std::cout << "高度范围: [" << config.min_height << ", " << config.max_height << "] 米" << std::endl;
    std::cout << "启用降采样: " << (config.enable_downsample ? "是" : "否") << std::endl;
    if (config.enable_downsample) {
        std::cout << "  体素大小: " << config.voxel_size << " 米" << std::endl;
    }
    std::cout << "去除离群点: " << (config.remove_outliers ? "是" : "否") << std::endl;
    if (config.remove_outliers) {
        std::cout << "  搜索半径: " << config.outlier_radius << " 米" << std::endl;
        std::cout << "  最小邻居数: " << config.outlier_min_neighbors << std::endl;
    }
    std::cout << "============================\n" << std::endl;
}
