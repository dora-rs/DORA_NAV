#pragma once
#include <vector>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

namespace hdl_graph_slam_dora {

struct KeyFrame;

class MapSaver {
public:
    MapSaver();
    void loadConfig(const YAML::Node& config);
    bool saveMap(const std::vector<std::shared_ptr<KeyFrame>>& keyframes);

private:
    std::string destination_path_;
    double map_cloud_resolution_;
};

} // namespace hdl_graph_slam_dora
