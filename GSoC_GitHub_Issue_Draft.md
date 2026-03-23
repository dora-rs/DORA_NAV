# GitHub Issue Draft — Copy-Paste Ready

**Post this at:** https://github.com/dora-rs/DORA_NAV/issues/new

---

## Title
```
[GSoC #8] Local Obstacle Avoidance Operator using Dynamic Window Approach (DWA)
```

---

## Body (copy everything below this line)

---

### Overview

This issue tracks the implementation of a **Local Planning and Obstacle Avoidance operator** for DORA_NAV as part of **GSoC Project #8**. The goal is to enable a mobile robot to perceive its immediate surroundings via LiDAR and generate real-time velocity commands (`v`, `ω`) to navigate around obstacles while following a global path.

---

### Proposed Algorithm — Dynamic Window Approach (DWA)

I plan to implement a simplified DWA local planner based on [Fox et al. 1997](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf).

**How DWA works:**
1. At each timestep, sample a set of `(v, ω)` velocity pairs within a dynamically constrained window (based on current speed and robot acceleration limits)
2. For each sampled velocity, simulate the robot's trajectory forward for a short horizon (~1-2 seconds)
3. Score each trajectory using an objective function:
   ```
   G(v, ω) = α·heading(v,ω) + β·clearance(v,ω) + γ·velocity(v,ω)
   ```
   - `heading` — how well the trajectory points toward the local goal
   - `clearance` — minimum distance to any obstacle along the trajectory
   - `velocity` — reward for forward progress
4. Select the `(v, ω)` with the highest score and publish it as `cmd_vel`

---

### New Node: `local_planner`

**Location:** `planning/local_planner/`

**DORA Inputs:**

| Input ID | Source | Type | Description |
|----------|--------|------|-------------|
| `pointcloud` | `mujoco_sim/pointcloud` | raw bytes | 16-beam LiDAR scan, each point: `[x, y, z, intensity]` as floats |
| `local_goal` | `planning/raw_path` | `float[]` | Next waypoint(s) from global planner in world frame |
| `cur_pose` | `mujoco_sim/ground_truth_pose` | `Pose2D_h` | Robot position `{float x, y, theta}` (theta in degrees) |

**DORA Output:**

| Output ID | Consumers | Type | Description |
|-----------|-----------|------|-------------|
| `cmd_vel` | `lat_controller`, `lon_controller` | `CmdVel` | `{float linear_v /*m/s*/, float angular_w /*rad/s*/}` |

**New struct to define:**
```cpp
struct CmdVel {
    float linear_v;   // forward velocity in m/s
    float angular_w;  // angular velocity in rad/s (+ = left)
};
```

---

### New Dataflow: `dataflow_local_avoidance.yml`

The new dataflow replaces the direct `planning → controllers` connection with a local planner in between:

**Before:**
```
mujoco_sim → planning → lat_controller
                      → lon_controller
```

**After:**
```
mujoco_sim → local_planner → lat_controller
planning   ↗               → lon_controller
```

---

### Simulation Environment Changes

- Add static obstacle boxes to `simulation/mujoco_bridge/models/robot_warehouse.xml`
- Test scenarios:
  - Point-to-point with 3–5 randomly placed box obstacles
  - Narrow corridor (two parallel walls, ~1.2m gap)
  - Dynamic obstacle (a moving body in MuJoCo, stretch goal)

---

### Testing Plan

**Unit tests** (GoogleTest) covering:
- Free space: `linear_v > 0`, `|angular_w|` small when path is clear
- Obstacle directly ahead: planner must turn, not drive forward
- Narrow corridor: robot passes through without collision
- Dead-end: robot rotates to escape (should not get stuck)

**Integration test:**
- Full `docker compose up` with `dataflow_local_avoidance.yml`
- Robot navigates from start to goal avoiding all obstacles
- Verified visually via Rerun viewer

---

### Deliverables Checklist

- [ ] `planning/local_planner/` — C++ DWA implementation as DORA node
- [ ] `dataflow_local_avoidance.yml` — new dataflow YAML
- [ ] Obstacle boxes added to MuJoCo warehouse model
- [ ] Unit tests with edge case coverage
- [ ] `README_local_planner.md` — quick-start guide with YAML config reference
- [ ] Recorded demo video of robot navigating with obstacle avoidance

---

### Configurable Parameters (YAML / env vars)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_LINEAR_V` | `0.5 m/s` | Maximum forward speed |
| `MAX_ANGULAR_W` | `1.0 rad/s` | Maximum turning speed |
| `MAX_LINEAR_ACCEL` | `0.3 m/s²` | Max acceleration (dynamic window) |
| `SAFETY_MARGIN` | `0.3 m` | Minimum clearance to obstacles |
| `GOAL_TOLERANCE` | `0.5 m` | Distance at which goal is considered reached |
| `PREDICT_HORIZON` | `1.5 s` | Trajectory simulation time |
| `DT` | `0.1 s` | Simulation timestep |

---

### Related Files in Existing Codebase

- `simulation/mujoco_bridge/src/mujoco_sim_bridge.cpp` — pointcloud format reference
- `control/vehicle_control/lat_controller/` — existing controller this node feeds into
- `planning/routing_planning/node_routing_core.cpp` — existing global planner (upstream of local planner)
- `include/SlamPose.h` — `Pose2D_h` struct definition

---

### Questions for Mentors

1. Should `cmd_vel` replace `SteeringCmd`/`TrqBreCmd` entirely, or should the local planner output in the existing format to keep the lat/lon controllers unchanged?
2. Is a pure DWA implementation sufficient, or would you prefer APF or a hybrid approach?
3. Should the local goal be a single next waypoint or a look-ahead window of the global path?

---

*This issue is part of GSoC 2025 Project #8 — Local Obstacle Avoidance for Dora Robots.*

---

## After Posting

Once the issue is live, post this comment:
```
@dora-bot assign me
```
