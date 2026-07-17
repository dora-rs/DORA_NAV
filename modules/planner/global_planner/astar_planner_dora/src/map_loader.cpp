#include "astar_planner/map_loader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>

namespace astar_planner {

bool loadPGM(const std::string& filename, MapInfo& map) {
    std::ifstream in(filename, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "Error: Cannot open PGM file: " << filename << std::endl;
        return false;
    }

    std::string header;
    in >> header;
    if (header != "P5") {
        std::cerr << "Error: Only P5 format PGM files (binary) are supported" << std::endl;
        return false;
    }

    while (in.peek() == '#') {
        std::string line;
        std::getline(in, line);
    }

    in >> map.width >> map.height;
    if (map.width <= 0 || map.height <= 0) {
        std::cerr << "Error: Invalid map dimensions: " << map.width << " x " << map.height << std::endl;
        return false;
    }

    int maxval;
    in >> maxval;
    in.get();

    map.data.resize(map.width * map.height);
    in.read(reinterpret_cast<char*>(map.data.data()), map.width * map.height);

    if (!in) {
        std::cerr << "Error: Failed to read PGM data" << std::endl;
        return false;
    }

    // 统计地图数据
    int min_val = 255, max_val = 0;
    std::map<int, int> value_counts;
    for (size_t i = 0; i < map.data.size(); i++) {
        int val = map.data[i];
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
        value_counts[val]++;
    }

    std::cout << "Successfully loaded PGM map: " << map.width << " x " << map.height << std::endl;
    std::cout << "Pixel value range: [" << min_val << ", " << max_val << "]" << std::endl;
    std::cout << "Top 5 pixel values:" << std::endl;

    std::vector<std::pair<int, int>> sorted_counts(value_counts.begin(), value_counts.end());
    std::sort(sorted_counts.begin(), sorted_counts.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    for (int i = 0; i < std::min(5, (int)sorted_counts.size()); i++) {
        std::cout << "  Value " << sorted_counts[i].first
                  << ": " << sorted_counts[i].second << " pixels" << std::endl;
    }

    return true;
}

bool loadMapYaml(const std::string& filename, MapInfo& map) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        std::cerr << "Warning: Cannot open YAML file: " << filename << std::endl;
        std::cerr << "Using default values: resolution=0.05, origin=(0, 0)" << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(in, line)) {
        std::istringstream iss(line);
        std::string key;
        if (!(iss >> key)) {
            continue;
        }

        if (key == "resolution:") {
            iss >> map.resolution;
        } else if (key == "origin:") {
            std::string token;
            iss >> token;

            size_t pos = token.find('[');
            if (pos != std::string::npos) {
                token = token.substr(pos + 1);
            }
            map.origin_x = std::stod(token);

            std::string y_token;
            iss >> y_token;
            if (!y_token.empty() && y_token.back() == ',') {
                y_token.pop_back();
            }
            map.origin_y = std::stod(y_token);
        }
    }

    std::cout << "Map parameters: resolution=" << map.resolution
              << " origin=(" << map.origin_x << ", " << map.origin_y << ")" << std::endl;
    return true;
}

MapInfo inflateMap(const MapInfo& map, double inflation_radius) {
    MapInfo inflated_map = map;

    // 计算膨胀半径对应的栅格数
    int inflation_cells = static_cast<int>(std::ceil(inflation_radius / map.resolution));

    std::cout << "Obstacle inflation: radius=" << inflation_radius << "m ("
              << inflation_cells << " cells)" << std::endl;

    // 创建临时地图存储膨胀结果
    std::vector<unsigned char> temp_data = map.data;

    const int OBSTACLE_THRESHOLD = 50;
    const unsigned char INFLATION_VALUE = 64;  // 膨胀区域使用深灰色

    // 遍历原始地图，找到所有障碍物并膨胀
    for (int y = 0; y < map.height; y++) {
        for (int x = 0; x < map.width; x++) {
            int idx = y * map.width + x;

            // 如果当前栅格是障碍物
            if (map.data[idx] <= OBSTACLE_THRESHOLD) {
                // 在膨胀半径内的所有栅格都标记为障碍物
                for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;

                        // 边界检查
                        if (nx < 0 || ny < 0 || nx >= map.width || ny >= map.height) {
                            continue;
                        }

                        // 计算实际距离
                        double dist = std::sqrt(dx * dx + dy * dy) * map.resolution;

                        // 如果在膨胀半径内
                        if (dist <= inflation_radius) {
                            int neighbor_idx = ny * map.width + nx;
                            // 只有当该栅格不是原始障碍物时，才标记为膨胀区域
                            if (map.data[neighbor_idx] > OBSTACLE_THRESHOLD) {
                                temp_data[neighbor_idx] = INFLATION_VALUE;  // 膨胀区域用深灰色
                            }
                        }
                    }
                }
            }
        }
    }

    inflated_map.data = temp_data;

    // 统计膨胀后的障碍物数量
    int original_obstacle_count = 0;
    int inflation_count = 0;
    for (size_t i = 0; i < inflated_map.data.size(); i++) {
        if (inflated_map.data[i] == 0) {
            original_obstacle_count++;
        } else if (inflated_map.data[i] == INFLATION_VALUE) {
            inflation_count++;
        }
    }
    std::cout << "Original obstacle cells: " << original_obstacle_count << std::endl;
    std::cout << "Inflation cells: " << inflation_count << std::endl;
    std::cout << "Total obstacle cells: " << (original_obstacle_count + inflation_count) << std::endl;

    return inflated_map;
}

} // namespace astar_planner
