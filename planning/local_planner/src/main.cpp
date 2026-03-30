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
                                float lookahead_dist = 2.0f)
{
    int num_floats  = static_cast<int>(data_len / sizeof(float));
    int num_points  = num_floats / 2;
    const float* xs = reinterpret_cast<const float*>(data);
    const float* ys = xs + num_points;

    if (num_points <= 0) return {1.0f, 0.0f};  // fallback

    static int print_cnt = 0;
    if (print_cnt++ % 10 == 0) {
        printf("[DEBUG] extract_local_goal: num_points=%d. First=(%.2f, %.2f) Last=(%.2f, %.2f)\n", 
               num_points, xs[0], ys[0], xs[num_points-1], ys[num_points-1]);
    }

    // 1. Find the closest point on the local path to the origin (robot is at 0,0 locally)
    int closest_idx = 0;
    float min_dist2 = std::numeric_limits<float>::max();
    for (int i = 0; i < num_points; ++i) {
        float dist2 = xs[i]*xs[i] + ys[i]*ys[i];
        if (dist2 < min_dist2) {
            min_dist2 = dist2;
            closest_idx = i;
        }
    }

    // 2. Walk FORWARD along the path to find the lookahead goal
    for (int i = closest_idx; i < num_points; ++i) {
        if (std::sqrt(xs[i]*xs[i] + ys[i]*ys[i]) >= lookahead_dist) {
            // Routing frame has Y as forward, X as right.
            // DWA frame needs X as forward, Y as left.
            return { ys[i], -xs[i] };
        }
    }
    
    // Default: last point in path if we are reaching the end
    return { ys[num_points - 1], -xs[num_points - 1] };
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
                    local_goal = extract_local_goal(data, data_len, planner.config().predict_time * planner.config().max_v + 1.0f);
                    has_path = true;
                }
            }

            // ── Timer tick → compute and publish cmd_vel ──────────────────
            else if (strncmp(input_id, "tick", 4) == 0) {
                CmdVel cmd = {0.0f, 0.0f};

                if (!has_pose || !has_path) {
                    // DEBUG: If missing inputs, force a slow forward crawl to test actuation
                    printf("[local_planner] Waiting for inputs (pose:%d path:%d). Forcing v=0.2\n", has_pose, has_path);
                    cmd.linear_v = 0.2f;
                    cmd.angular_w = 0.0f;
                } else {
                    // Normal DWA operation
                    cmd = planner.compute(robot_state,
                                                 local_goal,
                                                 obstacle_map.obstacles());
                }

                // Debug print the goal tracking
                static int tick_cnt = 0;
                if (++tick_cnt % 10 == 0) {
                    printf("[local_planner] Local goal is (%.2f, %.2f) mapped from path. Tracking active.\n",
                           local_goal.x, local_goal.y);
                }

                // Save commanded velocities as the current robot state for the NEXT tick
                // This allows the DWA dynamic window to accelerate over time.
                robot_state.v = cmd.linear_v;
                robot_state.w = cmd.angular_w;

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
