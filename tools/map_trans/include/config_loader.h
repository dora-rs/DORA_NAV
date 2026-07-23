#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <string>

// 配置结构体
struct Config {
    // 输入配置
    std::string pcd_file;

    // 输出配置
    std::string pgm_file;
    std::string yaml_file;

    // 地图参数
    double resolution;        // 栅格分辨率（米/像素）
    double min_height;        // 最小高度阈值（米）
    double max_height;        // 最大高度阈值（米）

    // 处理参数
    bool enable_downsample;   // 是否启用降采样
    double voxel_size;        // 降采样体素大小
    bool remove_outliers;     // 是否去除离群点
    double outlier_radius;    // 离群点搜索半径
    int outlier_min_neighbors; // 离群点最小邻居数

    // 默认构造函数，设置默认值
    Config();
};

// 配置加载器类
class ConfigLoader {
public:
    // 从JSON文件加载配置
    static bool loadConfig(const std::string& config_file, Config& config);

    // 验证配置的有效性
    static bool validateConfig(const Config& config);

    // 打印配置信息
    static void printConfig(const Config& config);
};

#endif // CONFIG_LOADER_H
