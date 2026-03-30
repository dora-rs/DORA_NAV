#include "local_planner.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <cstdio>

LocalPlanner::LocalPlanner(const DWAConfig& config) : config_(config) {}

void LocalPlanner::load_config_from_env()
{
    auto getf = [](const char* name, float def) -> float {
        const char* v = std::getenv(name);
        return v ? std::atof(v) : def;
    };
    config_.max_v        = getf("DWA_MAX_V",        config_.max_v);
    config_.min_v        = getf("DWA_MIN_V",        config_.min_v);
    config_.max_w        = getf("DWA_MAX_W",        config_.max_w);
    config_.max_accel_v  = getf("DWA_MAX_ACCEL_V",  config_.max_accel_v);
    config_.max_accel_w  = getf("DWA_MAX_ACCEL_W",  config_.max_accel_w);
    config_.predict_time = getf("DWA_PREDICT_TIME", config_.predict_time);
    config_.dt           = getf("DWA_DT",           config_.dt);
    config_.alpha        = getf("DWA_ALPHA",        config_.alpha);
    config_.beta         = getf("DWA_BETA",         config_.beta);
    config_.gamma        = getf("DWA_GAMMA",        config_.gamma);
    config_.robot_radius  = getf("DWA_ROBOT_RADIUS", config_.robot_radius);
    config_.safety_margin = getf("DWA_SAFETY_MARGIN",config_.safety_margin);
    config_.goal_tolerance= getf("DWA_GOAL_TOL",    config_.goal_tolerance);
}

bool LocalPlanner::goal_reached(const RobotState& state, const Goal& local_goal) const
{
    return std::sqrt(local_goal.x * local_goal.x + local_goal.y * local_goal.y) < config_.goal_tolerance;
}

CmdVel LocalPlanner::compute(const RobotState& state,
                              const Goal& goal,
                              const std::vector<Obstacle>& obstacles)
{
    if (goal_reached(state, goal)) {
        static int print_count = 0;
        if (print_count++ % 10 == 0) {
            printf("[DWA] GOAL REACHED! Goal distance is %.2f (tol: %.2f)\n",
                   std::sqrt(goal.x*goal.x + goal.y*goal.y), config_.goal_tolerance);
        }
        return {0.0f, 0.0f};
    }

    // Goal is ALREADY in the robot's local frame (from routing_planning)
    Goal local_goal = goal;

    // Initial state in local frame is origin, facing 0, with current v and w
    RobotState local_state = {0.0f, 0.0f, 0.0f, state.v, state.w};

    // Dynamic window: velocity window constrained by acceleration limits
    float v_min = std::max(config_.min_v, state.v - config_.max_accel_v * config_.dt);
    float v_max = std::min(config_.max_v, state.v + config_.max_accel_v * config_.dt);
    float w_min = std::max(-config_.max_w, state.w - config_.max_accel_w * config_.dt);
    float w_max = std::min( config_.max_w, state.w + config_.max_accel_w * config_.dt);

    float best_score = -std::numeric_limits<float>::infinity();
    CmdVel best_cmd  = {0.0f, 0.0f};
    bool   found     = false;

    for (float v = v_min; v <= v_max + 1e-6f; v += config_.v_resolution) {
        for (float w = w_min; w <= w_max + 1e-6f; w += config_.w_resolution) {
            float score = score_trajectory(v, w, local_state, local_goal, obstacles);
            if (score > best_score) {
                best_score = score;
                best_cmd   = {v, w};
                found      = true;
            }
        }
    }

    if (!found || best_cmd.linear_v < 0.01f) {
        // Recovery Mode: if DWA is stuck or commanding near zero velocity, check clearance
        float min_dist = min_obstacle_dist(0.0f, 0.0f, obstacles);
        if (min_dist < config_.robot_radius + 0.5f) { // dangerously close to something
            printf("[DWA] RECOVERY: Dist to obstacle (%.2f) critically low. Reversing!\n", min_dist);
            return {-0.15f, 0.0f}; // back up slowly straight
        } else {
            // It's stuck but NOT critically close to a wall! Log why.
            printf("[DWA] STUCK: v=0, but min_dist=%.2f. best_score=%.2f LocalGoal(%.2f, %.2f) Found=%d\n", 
                   min_dist, best_score, local_goal.x, local_goal.y, found);
        }
    }

    return found ? best_cmd : CmdVel{0.0f, 0.0f};
}

float LocalPlanner::score_trajectory(float v, float w,
                                      const RobotState& state,
                                      const Goal& goal,
                                      const std::vector<Obstacle>& obstacles) const
{
    RobotState s = state;
    float min_dist = std::numeric_limits<float>::infinity();
    int steps = static_cast<int>(config_.predict_time / config_.dt);

    for (int i = 0; i < steps; ++i) {
        s = step(s, v, w);
        float d = min_obstacle_dist(s.x, s.y, obstacles);
        if (d < config_.robot_radius + config_.safety_margin) {
            return -1.0f;  // collision — discard this trajectory
        }
        min_dist = std::min(min_dist, d);
    }

    // Scoring components (all normalised to [0,1] range)
    float h = heading_score(s, goal);
    float c = std::min(min_dist / 5.0f, 1.0f);  // clearance, cap at 5m
    float vv = v / config_.max_v;                 // velocity reward

    return config_.alpha * h + config_.beta * c + config_.gamma * vv;
}

RobotState LocalPlanner::step(const RobotState& s, float v, float w) const
{
    RobotState next = s;
    next.theta += w * config_.dt;
    next.x     += v * std::cos(next.theta) * config_.dt;
    next.y     += v * std::sin(next.theta) * config_.dt;
    next.v      = v;
    next.w      = w;
    return next;
}

float LocalPlanner::min_obstacle_dist(float px, float py,
                                       const std::vector<Obstacle>& obstacles) const
{
    float min_d = std::numeric_limits<float>::infinity();
    for (const auto& obs : obstacles) {
        float dx = px - obs.x;
        float dy = py - obs.y;
        float d  = std::sqrt(dx * dx + dy * dy);
        if (d < min_d) min_d = d;
    }
    return min_d;
}

float LocalPlanner::heading_score(const RobotState& end, const Goal& goal) const
{
    float dx    = goal.x - end.x;
    float dy    = goal.y - end.y;
    float angle = std::atan2(dy, dx);
    float diff  = std::abs(angle - end.theta);
    // Normalise to [0, pi]
    while (diff > M_PI)  diff -= 2.0f * M_PI;
    diff = std::abs(diff);
    return 1.0f - diff / M_PI;
}
