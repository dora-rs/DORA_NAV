#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>
#include <limits>

extern "C" {
#include "node_api.h"
}

struct Point2D { float x; float y; };
struct TwistCmd { float linear_x, linear_y, angular_z; };

// Kinematics
static const float MAX_V          = 1.0f;   
static const float MIN_V          = 0.1f;
static const float MAX_W          = 1.2f;   
static const float V_RESOLUTION   = 0.1f;   
static const float W_RESOLUTION   = 0.15f;  

// Simulation
static const float PREDICT_TIME   = 3.0f;
static const float DT             = 0.25f;  

// Safety & Scoring
static const float ROBOT_RADIUS   = 1.0f; 
static const float LOOKAHEAD      = 1.5f;   
static const float OBSTACLE_LIMIT = 2.5f;

// Tune Weights for DWA Scoring
static const float HEADING_COST   = 1.0f;   
static const float CLEARANCE_COST = 5.0f;   
static const float VELOCITY_COST  = 2.0f;   


std::vector<Point2D> obstacles;
std::vector<Point2D> local_path;
bool has_path=false, has_obs=false;

void handle_pointcloud(const char* data, size_t len) {
    obstacles.clear();
    size_t n = len / (3 * sizeof(float));
    const float* pts = reinterpret_cast<const float*>(data);
    
    for (size_t i = 0; i < n; ++i) {
        float lx = pts[i*3 + 0];
        float ly = pts[i*3 + 1];
        float lz = pts[i*3 + 2];

        if (lz < 0.05f || lz > 1.5f) continue; 
        if (std::hypot(lx, ly) > 3.5f) continue;

        obstacles.push_back({lx, ly});
    }
    has_obs = true;
}

void handle_path(const char* data, size_t len) {
    int n = (len / sizeof(float)) / 2;
    const float* arr = reinterpret_cast<const float*>(data);
    local_path.clear();
    
    for (int i = 0; i < n; ++i) {
        float raw_x = arr[i];       
        float raw_y = arr[i + n];   

        float local_x = raw_y;      
        float local_y = -raw_x;     

        local_path.push_back({local_x, local_y});
    }
    has_path = true;
}

TwistCmd compute_dwa() {
    float goal_x = local_path.back().x;
    float goal_y = local_path.back().y;
    
    for (int i = 0; i < (int)local_path.size(); ++i) {
        float dist = std::hypot(local_path[i].x, local_path[i].y);
        if (dist >= LOOKAHEAD) {
            goal_x = local_path[i].x;
            goal_y = local_path[i].y;
            break;
        }
    }

    float best_v = 0.0f;
    float best_w = 0.0f;
    float max_score = -std::numeric_limits<float>::max();

    for (float v = MIN_V; v <= MAX_V; v += V_RESOLUTION) {
        for (float w = -MAX_W; w <= MAX_W; w += W_RESOLUTION) {
            
            float sim_x = 0.0f;
            float sim_y = 0.0f;
            float sim_theta = 0.0f;
            float min_obs_dist = std::numeric_limits<float>::max();
            bool collision = false;

            for (float t = 0; t <= PREDICT_TIME; t += DT) {
                sim_theta += w * DT;
                sim_x += v * std::cos(sim_theta) * DT;
                sim_y += v * std::sin(sim_theta) * DT;

                for (const auto& obs : obstacles) {
                    float dist = std::hypot(sim_x - obs.x, sim_y - obs.y);
                    if (dist < min_obs_dist) min_obs_dist = dist;
                    if (dist <= ROBOT_RADIUS) {
                        collision = true;
                        break;
                    }
                }
                if (collision) break;
            }

            if (collision) continue;

            float angle_to_goal = std::atan2(goal_y - sim_y, goal_x - sim_x);
            float heading_error = std::abs(angle_to_goal - sim_theta);
            float heading_score = 1.0f - (heading_error / 3.14159f); 
            if (heading_score < 0.0f) heading_score = 0.0f;

            float clearance_score = min_obs_dist; 
            if (clearance_score > OBSTACLE_LIMIT) clearance_score = OBSTACLE_LIMIT;

            float velocity_score = v / MAX_V;

            float total_score = (HEADING_COST * heading_score) + 
                                (CLEARANCE_COST * clearance_score) + 
                                (VELOCITY_COST * velocity_score);

            if (total_score > max_score) {
                max_score = total_score;
                best_v = v;
                best_w = w;
            }
        }
    }

    if (max_score == -std::numeric_limits<float>::max()) {
        std::cout << "[DWA] TRAPPED! Rotating." << std::endl;
        return {0.0f, 0.0f, MAX_W * 0.5f}; 
    }

    return {best_v, 0.0f, best_w};
}

int main() {
    void* ctx = init_dora_context_from_env();
    std::cout << "[DWA Planner] Started Optimized Version." << std::endl;

    while (true) {
        void* event = dora_next_event(ctx);
        if (!event) break;
        
        if (read_dora_event_type(event) == DoraEventType_Input) {
            char *id, *data; size_t id_len, data_len;
            read_dora_input_id(event, &id, &id_len);
            read_dora_input_data(event, &data, &data_len);

            if      (strncmp(id, "pointcloud", id_len) == 0) handle_pointcloud(data, data_len);
            else if (strncmp(id, "raw_path",   id_len) == 0) handle_path(data, data_len);

            if (has_path && has_obs && strncmp(id, "raw_path", id_len) == 0) {
                TwistCmd cmd = compute_dwa();
                
                std::string out_id = "twist_cmd";
                dora_send_output(ctx, &out_id[0], out_id.size(), (char*)&cmd, sizeof(cmd));
            }
        }
        free_dora_event(event);
    }
    return 0;
}