#include "mujoco_sim_bridge.hpp"
#include "Controller.h"
#include <iostream>
#include <cstring>
#include <cmath>
#include <sys/time.h>

extern "C" {
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

namespace dora_sim {

MujocoSimBridge::MujocoSimBridge() = default;

MujocoSimBridge::~MujocoSimBridge() {
    stop();
    if (data_) {
        mj_deleteData(data_);
    }
    if (model_) {
        mj_deleteModel(model_);
    }
}

bool MujocoSimBridge::initialize(const SimConfig& config) {
    config_ = config;

    // Load Mujoco model
    char error[1000] = "";
    model_ = mj_loadXML(config.model_path.c_str(), nullptr, error, 1000);
    if (!model_) {
        std::cerr << "[MujocoSimBridge] Failed to load model: " << error << std::endl;
        return false;
    }

    // Create simulation data
    data_ = mj_makeData(model_);
    if (!data_) {
        std::cerr << "[MujocoSimBridge] Failed to create simulation data" << std::endl;
        return false;
    }

    // Set simulation timestep
    model_->opt.timestep = config.sim_timestep;

    // Load initial keyframe if available (sets robot start position/heading)
    if (model_->nkey > 0) {
        mj_resetDataKeyframe(model_, data_, 0);
        mj_forward(model_, data_);
        std::cout << "[MujocoSimBridge] Loaded keyframe 0 (initial pose)" << std::endl;
    }

    // Find robot body (assume first body after world is robot base)
    robot_body_id_ = 1;  // Default to first body
    for (int i = 1; i < model_->nbody; i++) {
        const char* name = mj_id2name(model_, mjOBJ_BODY, i);
        if (name && (strstr(name, "robot") || strstr(name, "base") || strstr(name, "chassis"))) {
            robot_body_id_ = i;
            std::cout << "[MujocoSimBridge] Found robot body: " << name << " (id=" << i << ")" << std::endl;
            break;
        }
    }

    // Find LiDAR body (or use robot body if not found)
    lidar_body_id_ = robot_body_id_;
    for (int i = 1; i < model_->nbody; i++) {
        const char* name = mj_id2name(model_, mjOBJ_BODY, i);
        if (name && (strstr(name, "lidar") || strstr(name, "laser"))) {
            lidar_body_id_ = i;
            std::cout << "[MujocoSimBridge] Found LiDAR body: " << name << " (id=" << i << ")" << std::endl;
            break;
        }
    }

    // Initialize LiDAR simulator
    LidarConfig lidar_config;

    const char* lidar_rays = std::getenv("LIDAR_RAYS");
    if (lidar_rays) {
        lidar_config.horizontal_rays = std::atoi(lidar_rays);
    }

    const char* lidar_range = std::getenv("LIDAR_RANGE");
    if (lidar_range) {
        lidar_config.max_range = std::atof(lidar_range);
    }

    const char* lidar_min_range = std::getenv("LIDAR_MIN_RANGE");
    if (lidar_min_range) {
        lidar_config.min_range = std::atof(lidar_min_range);
    }

    const char* lidar_beams = std::getenv("LIDAR_VERTICAL_BEAMS");
    if (lidar_beams) {
        lidar_config.vertical_beams = std::atoi(lidar_beams);
    }

    lidar_sim_ = std::make_unique<LidarSimulator>(lidar_config);

    // Initialize IMU simulator
    IMUConfig imu_config;

    const char* imu_noise = std::getenv("IMU_ADD_NOISE");
    if (imu_noise && std::strcmp(imu_noise, "0") == 0) {
        imu_config.add_noise = false;
    }

    imu_sim_ = std::make_unique<IMUSimulator>(imu_config);

    std::cout << "[MujocoSimBridge] Initialization complete" << std::endl;
    return true;
}

void MujocoSimBridge::run(void* dora_context) {
    running_ = true;

    while (running_) {
        void* event = dora_next_event(dora_context);
        if (event == nullptr) {
            break;
        }

        enum DoraEventType event_type = read_dora_event_type(event);

        if (event_type == DoraEventType_Input) {
            char* input_id;
            size_t input_id_len;
            read_dora_input_id(event, &input_id, &input_id_len);

            if (strncmp(input_id, "tick", 4) == 0) {
                // Timer tick - advance simulation

                // Calculate how many physics steps to run
                // Assuming tick is 10ms and physics step is 1ms
                int num_steps = 10;  // Adjust based on tick period
                stepSimulation(num_steps);

                // Send pointcloud if it's time
                if (shouldSendPointcloud(sim_time_)) {
                    auto pointcloud = lidar_sim_->generatePointCloud(model_, data_, lidar_body_id_);

                    std::string output_id = "pointcloud";
                    dora_send_output(
                        dora_context,
                        const_cast<char*>(output_id.c_str()),
                        output_id.length(),
                        reinterpret_cast<char*>(pointcloud.data()),
                        pointcloud.size()
                    );

                    last_pointcloud_time_ = sim_time_;
                }

                // Send IMU if it's time
                if (shouldSendIMU(sim_time_)) {
                    auto imu_msg = imu_sim_->generateIMU(model_, data_, robot_body_id_);
                    auto imu_data = imu_sim_->serialize(imu_msg);

                    std::string output_id = "imu_msg";
                    dora_send_output(
                        dora_context,
                        const_cast<char*>(output_id.c_str()),
                        output_id.length(),
                        reinterpret_cast<char*>(imu_data.data()),
                        imu_data.size()
                    );

                    last_imu_time_ = sim_time_;
                }

                // Send ground truth pose (theta in degrees for compatibility with Pose2D_h)
                // Pipeline yaw = atan2 angle (CCW from +X), same as mujoco
                auto pose = getGroundTruthPose();
                pose.theta = pose.theta * 180.0f / M_PI;
                std::string pose_id = "ground_truth_pose";
                dora_send_output(
                    dora_context,
                    const_cast<char*>(pose_id.c_str()),
                    pose_id.length(),
                    reinterpret_cast<char*>(&pose),
                    sizeof(pose)
                );

            } else if (strncmp(input_id, "twist_cmd", 9) == 0 ||
                       strncmp(input_id, "control_cmd", 11) == 0) {
                // Control input - apply to robot
                char* data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                processControlInput(data, data_len);

            } else if (strncmp(input_id, "SteeringCmd", 11) == 0) {
                char* data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                if (data_len >= sizeof(SteeringCmd_h)) {
                    const SteeringCmd_h* cmd = reinterpret_cast<const SteeringCmd_h*>(data);
                    steering_angle_ = cmd->SteeringAngle;
                    applyVehicleControl();
                }

            } else if (strncmp(input_id, "TrqBreCmd", 9) == 0) {
                char* data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                if (data_len >= sizeof(TrqBreCmd_h)) {
                    const TrqBreCmd_h* cmd = reinterpret_cast<const TrqBreCmd_h*>(data);
                    torque_pct_ = cmd->trq_value_3;
                    brake_value_ = cmd->bre_value;
                    trq_enable_ = cmd->trq_enable;
                    bre_enable_ = cmd->bre_enable;
                    applyVehicleControl();
                }
            }

        } else if (event_type == DoraEventType_Stop) {
            std::cout << "[MujocoSimBridge] Received stop event" << std::endl;
            running_ = false;
        }

        free_dora_event(event);
    }
}

void MujocoSimBridge::stop() {
    running_ = false;
}

void MujocoSimBridge::stepSimulation(int num_steps) {
    for (int i = 0; i < num_steps; i++) {
        mj_step(model_, data_);
        sim_time_ += model_->opt.timestep;
    }
}

void MujocoSimBridge::processControlInput(const char* data, size_t len) {
    // Expects twist command: [linear_x, linear_y, angular_z] as floats
    if (len >= 3 * sizeof(float)) {
        const float* cmd = reinterpret_cast<const float*>(data);
        float linear_x = cmd[0];
        float linear_z = cmd[2];  // angular velocity

        // Lookup specific actuators by name!
        int left_act = mj_name2id(model_, mjOBJ_ACTUATOR, "left_motor");
        int right_act = mj_name2id(model_, mjOBJ_ACTUATOR, "right_motor");

        if (left_act >= 0 && right_act >= 0) {
            float wheel_base = 0.5f;  // meters
            float left_vel = linear_x - linear_z * wheel_base / 2.0f;
            float right_vel = linear_x + linear_z * wheel_base / 2.0f;

            data_->ctrl[left_act] = left_vel;
            data_->ctrl[right_act] = right_vel;
            
            // Helpful debug log so we know it actually applied the velocity to MuJoCo
            printf("[MujocoSimBridge] Applied twist -> left: %.2f, right: %.2f\n", left_vel, right_vel);
        } else {
            printf("[MujocoSimBridge] ERROR: Could not find left_motor/right_motor actuators\n");
        }
    }
}

void MujocoSimBridge::applyVehicleControl() {
    if (model_->nu < 2) return;

    // Convert steering angle (degrees) + torque % to differential drive
    float linear_vel = 0.0f;
    if (trq_enable_ && !bre_enable_) {
        // Map torque/speed value to linear velocity (m/s)
        // run_speed is ~320 for the real vehicle, scale to ~0.5 m/s for sim robot
        linear_vel = torque_pct_ * 0.0015f;
    } else if (bre_enable_) {
        linear_vel = 0.0f;
    }

    // Convert steering angle to angular velocity
    // steering_angle_ is in degrees
    float steer_rad = steering_angle_ * (M_PI / 180.0f);
    float wheelbase = 0.5f;  // robot wheelbase in meters
    float angular_vel = 0.0f;
    if (std::fabs(linear_vel) > 0.01f) {
        angular_vel = linear_vel * std::tan(steer_rad) / wheelbase;
    }

    // Differential drive kinematics
    float left_vel  = linear_vel - angular_vel * wheelbase / 2.0f;
    float right_vel = linear_vel + angular_vel * wheelbase / 2.0f;

    data_->ctrl[0] = left_vel;
    data_->ctrl[1] = right_vel;
}

bool MujocoSimBridge::shouldSendPointcloud(double sim_time) const {
    double period = 1.0 / config_.lidar_output_rate_hz;
    return (sim_time - last_pointcloud_time_) >= period;
}

bool MujocoSimBridge::shouldSendIMU(double sim_time) const {
    double period = 1.0 / config_.imu_output_rate_hz;
    return (sim_time - last_imu_time_) >= period;
}

GroundTruthPose MujocoSimBridge::getGroundTruthPose() const {
    GroundTruthPose pose;

    // Get position from body
    pose.x = static_cast<float>(data_->xpos[robot_body_id_ * 3]);
    pose.y = static_cast<float>(data_->xpos[robot_body_id_ * 3 + 1]);

    // Get rotation matrix (row-major) and extract yaw
    // R = [cos(θ) -sin(θ) 0; sin(θ) cos(θ) 0; 0 0 1]
    // mat[0]=cos(θ), mat[3]=sin(θ)
    const mjtNum* mat = data_->xmat + robot_body_id_ * 9;
    pose.theta = static_cast<float>(std::atan2(mat[3], mat[0]));

    return pose;
}

}  // namespace dora_sim
