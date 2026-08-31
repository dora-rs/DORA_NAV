# DWA Local Planner

A reactive local obstacle avoidance and trajectory rollout node based on the Dynamic Window Approach (DWA). This version is optimized to run entirely in the robot's local coordinate frame, eliminating the need for global pose tracking within the control loop.

## I/O Interfaces
**Inputs:**
* `pointcloud` (Float Array): Raw 3D LiDAR hits. Filtered for floor/ceiling and evaluated in the local frame.
* `raw_path` (Float Array): The global breadcrumb trail. Transformed implicitly into local coordinates via axis swapping.

*Note: The control loop triggers dynamically upon the receipt of the `raw_path` message.*

**Outputs:**
* `twist_cmd` (Float Array): `[linear_x, linear_y, angular_z]` sent to the vehicle controller.

## Tuning & Parameters (PoC)
Currently, kinematic limits and scoring weights are hardcoded at the top of `local_planner.cpp`. 

**Key Weights & Limits:**
* `HEADING_COST` (1.0): Pulls the robot's simulated heading toward the lookahead point.
* `CLEARANCE_COST` (5.0): Heavy repulsion from LiDAR hits to prioritize safety.
* `VELOCITY_COST` (2.0): Preference for maintaining forward momentum.
* `ROBOT_RADIUS` (1.0m): Hard-stop collision radius.
* `PREDICT_TIME` (3.0s): The forward-simulation horizon.

*Note: Dynamic parameter loading via YAML will be introduced in a future update.*