# GSoC Project #8: Local Obstacle Avoidance for Dora Robots

**Size:** 175 hours | **Difficulty:** Medium | **Language:** C++  
**Mentors:** Min Cen, Yu Huang  
**Repo:** https://github.com/dora-rs/DORA_NAV

---

## What Already Exists in This Repo

Understanding what's built helps you know exactly what to add rather than rebuild.

| Component | Location | What it does |
|-----------|----------|-------------|
| Global planner | `planning/routing_planning/` | Frenet + A* path planning on a road map |
| Lateral controller | `control/vehicle_control/lat_controller/` | Follows a pre-computed path (steering) |
| Longitudinal controller | `control/vehicle_control/lon_controller/` | Speed / throttle control |
| MuJoCo simulation | `simulation/mujoco_bridge/` | Physics sim with fake LiDAR (16-beam, 1080 ray) |
| Localization | `localization/dora-hdl_localization/` | HDL-based point cloud localization |
| Visualization | `rerun/` | Streams point clouds, path, pose to Rerun viewer |

**What is missing (your job):** A **local planner** that sits between the global path and the controllers, reads live LiDAR data, detects obstacles, and outputs real-time velocity commands (`v`, `ω`) to dodge them.

---

## Deliverables Breakdown

---

### Deliverable 1 — Local Planning Operator (C++)

**What it is:** A new DORA node (`local_planner`) that runs at high frequency and outputs velocity commands.

**Algorithm choice (pick one):**

#### Option A — Dynamic Window Approach (DWA) [Recommended]
Samples velocity pairs `(v, ω)` in a "dynamic window" around current speed, simulates each trajectory for a short horizon, and picks the one that gets closest to the goal while staying clear of obstacles.

**Getting started:**
- Read the paper: [Fox et al. 1997 DWA](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)
- Understand the objective function: `G(v,ω) = σ(α·heading + β·dist + γ·velocity)`
- Look at `planning/routing_planning/node_routing_core.cpp` for how an existing planner node reads DORA inputs and sends outputs — yours will follow the same pattern
- Start with a 2D simulation (ignore Z), use the ground-truth pose from `mujoco_sim/ground_truth_pose`

#### Option B — Artificial Potential Field (APF) [Simpler to implement]
Robot is attracted toward the goal and repelled from obstacles. Sum the gradient vectors to get a velocity command.

**Getting started:**
- Repulsive force: `F_rep = η * (1/d - 1/d0) * (1/d²) * (p - p_obs)` for each obstacle within range `d0`
- Attractive force: `F_att = ζ * (p_goal - p_robot)`
- Watch out for local minima (the classic APF trap) — add a wall-following escape behavior

**Key files to create:**
```
planning/
└── local_planner/
    ├── CMakeLists.txt
    ├── include/
    │   ├── local_planner.hpp
    │   └── obstacle_map.hpp
    └── src/
        ├── main.cpp              ← DORA node boilerplate
        ├── local_planner.cpp     ← DWA or APF logic
        └── obstacle_map.cpp      ← converts LiDAR points → obstacle list
```

**DORA node template to follow:** Copy `simulation/mujoco_bridge/src/main.cpp` as starting boilerplate — it shows exactly how to init a DORA context, read inputs, and send outputs.

**Inputs your node will receive:**
```yaml
inputs:
  pointcloud: mujoco_sim/pointcloud       # raw LiDAR scan
  local_goal: planning/raw_path           # next waypoint from global planner
  cur_pose: mujoco_sim/ground_truth_pose  # robot x, y, theta
```

**Output your node will send:**
```yaml
outputs:
  - cmd_vel   # struct: { float linear_v; float angular_w; }
```

---

### Deliverable 2 — Navigation Dataflow (`dataflow_local_avoidance.yml`)

**What it is:** A new YAML file wiring your local planner into the pipeline, replacing the direct connection between the global planner and the controllers.

**Before (current flow):**
```
planning → lat_controller (path following only, no obstacle awareness)
planning → lon_controller
```

**After (your new flow):**
```
planning  → local_planner → cmd_vel_to_control → lat_controller
mujoco_sim ↗                                    → lon_controller
```

**Getting started:**
- Copy `dataflow_full_sim.yml` to `dataflow_local_avoidance.yml`
- Add your `local_planner` node section
- You may need a thin adapter node (`cmd_vel_to_control`) that converts `{v, ω}` into the `SteeringCmd_h` and `TrqBreCmd_h` structs the existing controllers expect — or modify the controllers directly

**Reference:** Look at `dataflow_full_sim.yml` lines 86–106 for how inputs/outputs/env vars are declared.

---

### Deliverable 3 — Simulation Demo

**What it is:** A recorded video or live demo of the robot navigating point-to-point while avoiding obstacles placed in the MuJoCo scene.

**Getting started:**
- Add static obstacle boxes to `simulation/mujoco_bridge/models/robot_warehouse.xml` (MuJoCo XML `<body>` + `<geom>` elements)
- Test narrow corridor scenario: place two walls with a ~1m gap
- Record using `rerun` output — open the Rerun viewer and screenshot/screenrecord

**Obstacle XML example to add to the model:**
```xml
<body name="obstacle_1" pos="5.0 3.0 0.5">
    <geom type="box" size="0.5 0.5 0.5" rgba="1 0 0 1"/>
</body>
```

---

### Deliverable 4 — Unit Tests

**What it is:** C++ tests validating your planner's collision-free logic in edge cases.

**Cases to cover:**
- Free space: robot should move toward goal with `v > 0`
- Obstacle directly ahead: robot should turn, not drive forward
- Narrow corridor: robot should pass through without collision
- Dead-end trap: robot should back up or rotate to escape

**Getting started:**
- Use [GoogleTest](https://github.com/google/googletest): `find_package(GTest REQUIRED)` in CMakeLists
- Test the planner logic in isolation — pass in fake obstacle lists and fake pose, check output `cmd_vel` is safe

---

### Deliverable 5 — Quick-Start Guide (`README_local_planner.md`)

**What it is:** Documentation so others can build and run your operator.

**Must include:**
- How to build: `cd planning/local_planner && mkdir build && cd build && cmake .. && make`
- How to run: `docker compose up` with your new dataflow
- YAML config parameters table (max speed, safety margin, goal tolerance, etc.)
- Data schema for input/output messages (struct field names and units)

---

## Suggested Implementation Order

```
Week 1-2:   Learn DWA/APF theory + read existing node code
Week 3-4:   Build obstacle_map from LiDAR pointcloud
Week 5-6:   Implement core DWA/APF logic (no DORA yet, test standalone)
Week 7-8:   Wrap in DORA node + write dataflow YAML
Week 9-10:  Add obstacles to MuJoCo model + test in sim
Week 11-12: Unit tests + edge case fixing
Week 13-14: Record demo + write documentation
```

---

## Key Data Structures to Know

```cpp
// Existing — pose from mujoco_sim (in include/SlamPose.h)
struct Pose2D_h {
    float x;      // meters
    float y;      // meters
    float theta;  // degrees (already converted by mujoco_sim node)
};

// Existing — steering command (in control/include/)
struct SteeringCmd_h {
    float SteeringAngle;  // degrees
};

// New — you will define this
struct CmdVel {
    float linear_v;   // m/s  forward velocity
    float angular_w;  // rad/s  angular velocity (+ = left turn)
};
```

---

## Useful References

- [DWA paper (Fox 1997)](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)
- [DORA node C API docs](https://dora-rs.ai/docs)
- [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [Rerun C++ logging](https://ref.rerun.io/docs/cpp/)
- Existing node to copy as template: `simulation/mujoco_bridge/src/main.cpp`
- Existing CMakeLists to copy: `control/vehicle_control/lat_controller/CMakeLists.txt`
