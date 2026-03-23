extern "C" {
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include "local_planner.hpp"
#include "obstacle_map.hpp"
#include "SlamPose.h"
#include <cstring>
#include <cstdio>
#include <cmath>

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Extract the next waypoint from the global planner's raw_path output.
/// raw_path is a float array: first half = x coords, second half = y coords.
/// We take the look-ahead index (e.g. 5th point) as the local goal.
static Goal extract_local_goal(const char* data, size_t data_len,
                                float robot_x, float robot_y,
                                float lookahead_dist = 3.0f)
{
    int num_floats  = static_cast<int>(data_len / sizeof(float));
    int num_points  = num_floats / 2;
    const float* xs = reinterpret_cast<const float*>(data);
    const float* ys = xs + num_points;

    if (num_points <= 0) return {robot_x + 1.0f, robot_y};  // fallback

    // Walk along path points and pick first one beyond lookahead_dist
    for (int i = 0; i < num_points; ++i) {
        float dx = xs[i] - robot_x;
        float dy = ys[i] - robot_y;
        if (std::sqrt(dx * dx + dy * dy) >= lookahead_dist) {
            return {xs[i], ys[i]};
        }
    }
    // Default: last point in path
    return {xs[num_points - 1], ys[num_points - 1]};
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main()
{
    printf("[local_planner] Starting DWA local obstacle avoidance node\n");

    void* dora_context = init_dora_context_from_env();
    if (!dora_context) {
        fprintf(stderr, "[local_planner] ERROR: failed to init DORA context\n");
        return -1;
    }

    // Initialise planner and load config from env vars
    LocalPlanner planner;
    planner.load_config_from_env();
    printf("[local_planner] DWA config loaded. max_v=%.2f max_w=%.2f\n",
           planner.config().max_v, planner.config().max_w);

    ObstacleMap obstacle_map;

    // Running state
    RobotState robot_state = {0, 0, 0, 0, 0};
    Goal       local_goal  = {1.0f, 0.0f};
    bool       has_pose    = false;
    bool       has_path    = false;

    // Event loop
    while (true) {
        void* event = dora_next_event(dora_context);
        if (!event) {
            fprintf(stderr, "[local_planner] ERROR: unexpected end of events\n");
            break;
        }

        enum DoraEventType event_type = read_dora_event_type(event);

        if (event_type == DoraEventType_Input) {
            char*  data;
            size_t data_len;
            char*  input_id;
            size_t input_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &input_id, &input_id_len);

            // ── Pointcloud → obstacle map ─────────────────────────────────
            if (strncmp(input_id, "pointcloud", 10) == 0) {
                int32_t num_points = (static_cast<int32_t>(data_len) - 16) / 16;
                obstacle_map.update(data, num_points);
            }

            // ── Current pose → robot state ────────────────────────────────
            else if (strncmp(input_id, "cur_pose", 8) == 0) {
                if (data_len >= sizeof(Pose2D_h)) {
                    const Pose2D_h* p = reinterpret_cast<const Pose2D_h*>(data);
                    robot_state.x     = p->x;
                    robot_state.y     = p->y;
                    // theta in mujoco_sim is already in degrees; convert to radians
                    robot_state.theta = p->theta * (M_PI / 180.0f);
                    has_pose = true;
                }
            }

            // ── Global path → local goal ──────────────────────────────────
            else if (strncmp(input_id, "local_goal", 10) == 0) {
                if (data_len > 0 && has_pose) {
                    local_goal = extract_local_goal(data, data_len,
                                                    robot_state.x, robot_state.y);
                    has_path = true;
                }
            }

            // ── Timer tick → compute and publish cmd_vel ──────────────────
            else if (strncmp(input_id, "tick", 4) == 0) {
                if (!has_pose || !has_path) {
                    free_dora_event(event);
                    continue;
                }

                CmdVel cmd = planner.compute(robot_state,
                                             local_goal,
                                             obstacle_map.obstacles());

                // Publish as twist_cmd: [linear_x, linear_y, angular_z]
                // mujoco_sim already has a handler for this format
                float twist[3] = {cmd.linear_v, 0.0f, cmd.angular_w};
                const char* out_id = "twist_cmd";
                dora_send_output(dora_context,
                                 const_cast<char*>(out_id), strlen(out_id),
                                 reinterpret_cast<char*>(twist), sizeof(twist));

                printf("[local_planner] twist_cmd: v=%.3f w=%.3f | obstacles=%zu\n",
                       cmd.linear_v, cmd.angular_w, obstacle_map.obstacles().size());
            }

        } else if (event_type == DoraEventType_Stop) {
            printf("[local_planner] Received stop event\n");
            free_dora_event(event);
            break;
        }

        free_dora_event(event);
    }

    free_dora_context(dora_context);
    printf("[local_planner] Exiting\n");
    return 0;
}
