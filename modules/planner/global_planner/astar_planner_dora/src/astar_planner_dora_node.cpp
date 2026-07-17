#include <iostream>
#include <string>
#include <optional>
#include <memory>
#include <fstream>

#include "astar_planner/astar_algorithm.hpp"
#include "astar_planner/map_loader.hpp"
#include "astar_planner/coordinate_transform.hpp"
#include "astar_planner/path_smoother.hpp"
#include "json_serialization.h"

extern "C" {
#include "node_api.h"
}

using namespace astar_planner;

// ==================== Debug 输出配置 ====================
// 每次规划后同时保存原始 A* 路径和平滑后路径，便于对比验证
// 生产环境可将下面两行注释掉
const std::string DEBUG_RAW_FILE      = "debug_path_raw.txt";
const std::string DEBUG_SMOOTHED_FILE = "debug_path_smoothed.txt";
// =========================================================

class AStarPlannerDoraNode {
public:
    AStarPlannerDoraNode(void* dora_context)
        : dora_context_(dora_context),
          path_seq_(0),
          map_loaded_(false),
          path_planned_(false) {
    }

    ~AStarPlannerDoraNode() {
        if (dora_context_) {
            free_dora_context(dora_context_);
        }
    }

    bool initialize() {
        std::cout << "=== A* Planner Dora Node Initialization ===" << std::endl;

        std::string pgm_file = "../maps/pgm/map.pgm";
        std::string yaml_file = "../maps/pgm/map.yaml";

        if (!loadPGM(pgm_file, map_)) {
            std::cerr << "Failed to load PGM map" << std::endl;
            return false;
        }

        loadMapYaml(yaml_file, map_);

        // 障碍物膨胀处理
        const double INFLATION_RADIUS = 0.2;  // 膨胀半径 0.2 米
        map_ = inflateMap(map_, INFLATION_RADIUS);

        map_loaded_ = true;
        std::cout << "Initialization completed successfully" << std::endl;
        return true;
    }

    void run() {
        if (!map_loaded_) {
            std::cerr << "Node not initialized" << std::endl;
            return;
        }

        std::cout << "Starting event loop..." << std::endl;

        while (true) {
            void* event = dora_next_event(dora_context_);
            if (!event) {
                std::cerr << "Unexpected end of event" << std::endl;
                break;
            }

            DoraEventType event_type = read_dora_event_type(event);

            if (event_type == DoraEventType_Input) {
                char* data;
                size_t data_len;
                char* id;
                size_t id_len;

                read_dora_input_data(event, &data, &data_len);
                read_dora_input_id(event, &id, &id_len);

                std::string topic_id(id, id_len);

                if (topic_id == "pose") {
                    handlePoseInput(data, data_len);
                } else if (topic_id == "goal") {
                    handleGoalInput(data, data_len);
                }
            }
            else if (event_type == DoraEventType_Stop) {
                std::cout << "Received stop event" << std::endl;
                free_dora_event(event);
                break;
            }

            free_dora_event(event);
        }

        std::cout << "Event loop finished" << std::endl;
    }

private:
    void handlePoseInput(const char* data, size_t len) {
        try {
            std::string json_str(data, len);
            PoseData pose;

            if (!parsePoseJson(json_str, pose)) {
                return;
            }

            current_pose_ = WorldPoint(pose.x, pose.y);
            // std::cout << "Received pose: (" << pose.x << ", " << pose.y << ")" << std::endl;

            // 全局路径只在收到 goal 时规划一次，pose 更新不触发重新规划
        } catch (const std::exception& e) {
            std::cerr << "Error processing pose: " << e.what() << std::endl;
        }
    }

    void handleGoalInput(const char* data, size_t len) {
        try {
            std::string json_str(data, len);
            GoalData goal;

            if (!parseGoalJson(json_str, goal)) {
                return;
            }

            current_goal_ = WorldPoint(goal.x, goal.y);
            path_planned_ = false;  // 新 goal 到来，重置规划标志
            std::cout << "Received new goal: (" << goal.x << ", " << goal.y << ")" << std::endl;

            if (current_pose_.has_value()) {
                planPath();
            } else {
                std::cout << "Waiting for pose before planning..." << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error processing goal: " << e.what() << std::endl;
        }
    }

    void planPath() {
        if (!current_pose_.has_value() || !current_goal_.has_value()) {
            return;
        }

        if (path_planned_) {
            std::cout << "Path already planned for current goal, skipping." << std::endl;
            return;
        }

        std::cout << "Planning path from (" << current_pose_->x << ", " << current_pose_->y
                  << ") to (" << current_goal_->x << ", " << current_goal_->y << ")" << std::endl;

        GridPoint start = worldToGrid(current_pose_->x, current_pose_->y, map_);
        GridPoint goal = worldToGrid(current_goal_->x, current_goal_->y, map_);

        auto start_opt = findNearestFreeCell(start, map_);
        auto goal_opt = findNearestFreeCell(goal, map_);

        if (!start_opt) {
            std::cerr << "ERROR: Cannot find valid start position near ("
                      << start.x << ", " << start.y << ")" << std::endl;
            publishEmptyPath();
            return;
        }

        if (!goal_opt) {
            std::cerr << "ERROR: Cannot find valid goal position near ("
                      << goal.x << ", " << goal.y << ")" << std::endl;
            publishEmptyPath();
            return;
        }

        auto path_opt = runAStar(map_, *start_opt, *goal_opt);

        if (!path_opt) {
            std::cerr << "ERROR: A* failed to find path" << std::endl;
            publishEmptyPath();
            return;
        }

        std::vector<WorldPoint> world_path;
        for (const auto& gp : *path_opt) {
            world_path.push_back(gridToWorld(gp.x, gp.y, map_));
        }

        // 平滑流水线：① Shortcut 可视性剪枝 → ② 等弧长重采样（间距 0.3m）
        const double RESAMPLE_INTERVAL = 0.3;  // 单位：米，可按需调整
        std::vector<WorldPoint> smoothed_path = smoothPath(world_path, map_, RESAMPLE_INTERVAL);

        // 同时保存原始路径和平滑路径，便于对比验证
        savePathToFile(world_path,    DEBUG_RAW_FILE,      "Raw A* Path");
        savePathToFile(smoothed_path, DEBUG_SMOOTHED_FILE, "Smoothed Path");

        path_planned_ = true;  // 标记本次 goal 已规划完成
        publishPath(smoothed_path);
    }

    void publishPath(const std::vector<WorldPoint>& world_path) {
        double timestamp = getCurrentTimestamp();
        nlohmann::json json_data = pathToJson(world_path, timestamp, path_seq_++);

        std::string json_str = json_data.dump();
        std::string topic = "path";

        int result = dora_send_output(dora_context_,
                                      const_cast<char*>(topic.c_str()),
                                      topic.length(),
                                      const_cast<char*>(json_str.c_str()),
                                      json_str.length());

        if (result != 0) {
            std::cerr << "Failed to publish path" << std::endl;
        } else {
            std::cout << "Published path with " << world_path.size() << " waypoints" << std::endl;
        }
    }

    // label 用于文件头注释，区分 Raw / Smoothed
    void savePathToFile(const std::vector<WorldPoint>& path,
                        const std::string& filename,
                        const std::string& label) {
        std::ofstream outfile(filename);
        if (!outfile.is_open()) {
            std::cerr << "Failed to open output file: " << filename << std::endl;
            return;
        }

        outfile << "# " << label << "\n";
        if (current_pose_.has_value()) {
            outfile << "# Start: (" << current_pose_->x << ", " << current_pose_->y << ")\n";
        }
        if (current_goal_.has_value()) {
            outfile << "# Goal:  (" << current_goal_->x << ", " << current_goal_->y << ")\n";
        }
        outfile << "# Total waypoints: " << path.size() << "\n";
        outfile << "# Format: x y\n";
        outfile << "\n";

        for (const auto& wp : path) {
            outfile << wp.x << " " << wp.y << "\n";
        }

        outfile.close();
        std::cout << label << " saved to: " << filename
                  << " (" << path.size() << " points)" << std::endl;
    }


    void publishEmptyPath() {
        double timestamp = getCurrentTimestamp();
        nlohmann::json json_data;
        json_data["header"]["timestamp"] = timestamp;
        json_data["header"]["seq"] = path_seq_++;
        json_data["poses"] = nlohmann::json::array();
        json_data["error"] = true;

        std::string json_str = json_data.dump();
        std::string topic = "path";

        dora_send_output(dora_context_,
                        const_cast<char*>(topic.c_str()),
                        topic.length(),
                        const_cast<char*>(json_str.c_str()),
                        json_str.length());
    }

    void* dora_context_;
    MapInfo map_;
    std::optional<WorldPoint> current_pose_;
    std::optional<WorldPoint> current_goal_;
    uint32_t path_seq_;
    bool map_loaded_;
    bool path_planned_;  // 当前 goal 是否已完成规划，收到新 goal 时重置
};

int main() {
    std::cout << "A* Planner Dora Node Starting..." << std::endl;

    void* dora_context = init_dora_context_from_env();
    if (!dora_context) {
        std::cerr << "Failed to initialize dora context" << std::endl;
        return 1;
    }

    AStarPlannerDoraNode node(dora_context);

    if (!node.initialize()) {
        std::cerr << "Failed to initialize node" << std::endl;
        return 1;
    }

    node.run();

    std::cout << "A* Planner Dora Node Exiting..." << std::endl;
    return 0;
}
