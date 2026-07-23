#include "json_serialization.h"
#include <iostream>
#include <chrono>

namespace astar_planner {

bool parsePoseJson(const std::string& json_str, PoseData& pose) {
    try {
        nlohmann::json json_data = nlohmann::json::parse(json_str);

        if (!json_data.contains("header") || !json_data.contains("pose")) {
            std::cerr << "Error: Missing required fields in pose JSON" << std::endl;
            return false;
        }

        pose.timestamp = json_data["header"]["timestamp"].get<double>();
        pose.x = json_data["pose"]["position"]["x"].get<double>();
        pose.y = json_data["pose"]["position"]["y"].get<double>();
        pose.z = json_data["pose"]["position"]["z"].get<double>();
        pose.qw = json_data["pose"]["orientation"]["w"].get<double>();
        pose.qx = json_data["pose"]["orientation"]["x"].get<double>();
        pose.qy = json_data["pose"]["orientation"]["y"].get<double>();
        pose.qz = json_data["pose"]["orientation"]["z"].get<double>();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing pose JSON: " << e.what() << std::endl;
        return false;
    }
}

bool parseGoalJson(const std::string& json_str, GoalData& goal) {
    try {
        nlohmann::json json_data = nlohmann::json::parse(json_str);

        if (!json_data.contains("x") || !json_data.contains("y")) {
            std::cerr << "Error: Missing required fields in goal JSON" << std::endl;
            return false;
        }

        goal.x = json_data["x"].get<double>();
        goal.y = json_data["y"].get<double>();
        goal.yaw = json_data.value("yaw", 0.0);

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing goal JSON: " << e.what() << std::endl;
        return false;
    }
}

nlohmann::json pathToJson(const std::vector<WorldPoint>& path,
                          double timestamp,
                          uint32_t seq) {
    nlohmann::json json_data;

    json_data["header"]["timestamp"] = timestamp;
    json_data["header"]["seq"] = seq;

    json_data["poses"] = nlohmann::json::array();
    for (const auto& wp : path) {
        nlohmann::json pose;
        pose["x"] = wp.x;
        pose["y"] = wp.y;
        json_data["poses"].push_back(pose);
    }

    return json_data;
}

double getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration);
    return seconds.count();
}

} // namespace astar_planner
