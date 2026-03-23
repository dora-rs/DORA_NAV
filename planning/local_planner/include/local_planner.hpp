#pragma once
#include "obstacle_map.hpp"
#include <cmath>
#include <vector>

/// Output velocity command
struct CmdVel {
    float linear_v;   // forward velocity (m/s)
    float angular_w;  // angular velocity (rad/s), + = left turn
};

/// Robot state snapshot
struct RobotState {
    float x;      // metres (world frame)
    float y;      // metres (world frame)
    float theta;  // radians (world frame, + = CCW)
    float v;      // current linear velocity (m/s)
    float w;      // current angular velocity (rad/s)
};

/// Local goal in world frame
struct Goal {
    float x;
    float y;
};

/// DWA tuning parameters — all configurable via env vars at runtime
struct DWAConfig {
    // Velocity limits
    float max_v        = 0.5f;   // m/s
    float min_v        = 0.0f;   // m/s  (no reversing by default)
    float max_w        = 1.0f;   // rad/s
    float max_accel_v  = 0.3f;   // m/s²
    float max_accel_w  = 0.6f;   // rad/s²

    // Sampling resolution
    float v_resolution = 0.05f;  // m/s per sample
    float w_resolution = 0.1f;   // rad/s per sample

    // Trajectory simulation
    float dt            = 0.1f;  // seconds per step
    float predict_time  = 1.5f;  // total horizon (seconds)

    // Scoring weights (must sum to ~1 for interpretability, but not required)
    float alpha = 0.15f;  // heading score weight
    float beta  = 1.0f;   // clearance score weight
    float gamma = 0.1f;   // velocity score weight

    // Safety
    float robot_radius    = 0.3f;  // metres — robot footprint radius
    float safety_margin   = 0.05f; // extra clearance on top of robot_radius
    float goal_tolerance  = 0.5f;  // metres — goal reached threshold
};

/// Dynamic Window Approach local planner.
/// Call compute() at each timestep to get a velocity command.
class LocalPlanner {
public:
    explicit LocalPlanner(const DWAConfig& config = DWAConfig{});

    /// Load config values from environment variables (called once at startup).
    void load_config_from_env();

    /// Compute the best (v, w) command for this timestep.
    /// Returns {0,0} if no safe trajectory found.
    CmdVel compute(const RobotState& state,
                   const Goal& goal,
                   const std::vector<Obstacle>& obstacles);

    /// Returns true if the robot is within goal_tolerance of the goal.
    bool goal_reached(const RobotState& state, const Goal& goal) const;

    const DWAConfig& config() const { return config_; }

private:
    DWAConfig config_;

    /// Score a single trajectory defined by (v, w).
    // Returns -1.0 if the trajectory collides.
    float score_trajectory(float v, float w,
                           const RobotState& state,
                           const Goal& goal,
                           const std::vector<Obstacle>& obstacles) const;

    /// Simulate one step of unicycle kinematics.
    RobotState step(const RobotState& s, float v, float w) const;

    /// Minimum distance from a trajectory point to any obstacle.
    float min_obstacle_dist(float px, float py,
                            const std::vector<Obstacle>& obstacles) const;

    /// Heading score: 1 - (angle_diff / pi), higher = more aligned with goal.
    float heading_score(const RobotState& end, const Goal& goal) const;
};
