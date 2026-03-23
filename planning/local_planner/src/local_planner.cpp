#include "local_planner.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

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

bool LocalPlanner::goal_reached(const RobotState& state, const Goal& goal) const
{
    float dx = goal.x - state.x;
    float dy = goal.y - state.y;
    return std::sqrt(dx * dx + dy * dy) < config_.goal_tolerance;
}

CmdVel LocalPlanner::compute(const RobotState& state,
                              const Goal& goal,
                              const std::vector<Obstacle>& obstacles)
{
    if (goal_reached(state, goal)) {
        return {0.0f, 0.0f};
    }

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
            float score = score_trajectory(v, w, state, goal, obstacles);
            if (score > best_score) {
                best_score = score;
                best_cmd   = {v, w};
                found      = true;
            }
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
