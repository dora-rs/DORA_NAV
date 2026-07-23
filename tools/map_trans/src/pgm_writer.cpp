#include "pgm_writer.h"
#include <fstream>
#include <iostream>
#include <filesystem>

bool PGMWriter::savePGM(const GridMap& map, const std::string& filename) {
    // 创建输出目录（如果不存在）
    std::filesystem::path file_path(filename);
    std::filesystem::path dir_path = file_path.parent_path();
    if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
        std::filesystem::create_directories(dir_path);
    }

    // 打开文件（二进制模式）
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "错误: 无法创建PGM文件: " << filename << std::endl;
        return false;
    }

    // 写入PGM文件头（P5格式）
    file << "P5\n";
    file << map.getWidth() << " " << map.getHeight() << "\n";
    file << "255\n";

    // 写入图像数据
    // 注意：PGM图像坐标系原点在左上角，需要Y轴翻转
    const auto& data = map.getData();
    int width = map.getWidth();
    int height = map.getHeight();

    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            int index = y * width + x;
            file.put(static_cast<char>(data[index]));
        }
    }

    file.close();

    std::cout << "成功保存PGM文件: " << filename << std::endl;
    return true;
}

bool PGMWriter::saveYAML(const GridMap& map,
                        const std::string& pgm_filename,
                        const std::string& yaml_filename) {
    // 创建输出目录（如果不存在）
    std::filesystem::path file_path(yaml_filename);
    std::filesystem::path dir_path = file_path.parent_path();
    if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
        std::filesystem::create_directories(dir_path);
    }

    // 打开文件
    std::ofstream file(yaml_filename);
    if (!file.is_open()) {
        std::cerr << "错误: 无法创建YAML文件: " << yaml_filename << std::endl;
        return false;
    }

    // 提取PGM文件名（不含路径）
    std::filesystem::path pgm_path(pgm_filename);
    std::string pgm_name = pgm_path.filename().string();

    // 写入YAML内容
    file << "image: " << pgm_name << "\n";
    file << "resolution: " << map.getResolution() << "\n";
    file << "origin: [" << map.getOriginX() << ", "
         << map.getOriginY() << ", 0.0]\n";
    file << "occupied_thresh: 0.65\n";
    file << "free_thresh: 0.196\n";
    file << "negate: 0\n";

    file.close();

    std::cout << "成功保存YAML文件: " << yaml_filename << std::endl;
    return true;
}
