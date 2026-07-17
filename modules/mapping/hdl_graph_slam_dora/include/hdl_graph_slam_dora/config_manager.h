#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>

namespace hdl_graph_slam_dora {

class ConfigManager {
public:
    bool loadFromFile(const std::string& config_path) {
        try {
            config_ = YAML::LoadFile(config_path);
            std::cout << "[Config] Loaded from: " << config_path << std::endl;
            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "[Config] YAML parse error: " << e.what() << std::endl;
            return false;
        }
    }

    YAML::Node getNode(const std::string& key) const {
        return config_[key];
    }

    template<typename T>
    T get(const std::string& key, const T& default_value) const {
        try {
            return config_[key].as<T>();
        } catch (...) {
            return default_value;
        }
    }

private:
    YAML::Node config_;
};

} // namespace hdl_graph_slam_dora
