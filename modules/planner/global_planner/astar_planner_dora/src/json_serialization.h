#ifndef JSON_SERIALIZATION_H
#define JSON_SERIALIZATION_H

#include "astar_planner/astar_algorithm.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

namespace astar_planner {

struct PoseData {
    double timestamp;
    double x, y, z;
    double qw, qx, qy, qz;
};

struct GoalData {
    double x, y;
    double yaw;
};

bool parsePoseJson(const std::string& json_str, PoseData& pose);

bool parseGoalJson(const std::string& json_str, GoalData& goal);

nlohmann::json pathToJson(const std::vector<WorldPoint>& path,
                          double timestamp,
                          uint32_t seq);

double getCurrentTimestamp();

} // namespace astar_planner

#endif // JSON_SERIALIZATION_H
