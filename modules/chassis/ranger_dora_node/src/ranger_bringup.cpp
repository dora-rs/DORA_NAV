/**
 * ranger_bringup.cpp
 *
 * Ranger底盘 Dora 节点
 *
 * 输入:
 *   timer    ← dora/timer/millis/10
 *   cmd_vel  ← planner/cmd_vel
 *             格式: { "linear":{"x":...}, "angular":{"z":...} }
 *
 * 输出:
 *   Odometry → ranger_miniv3/Odometry
 *             格式: { "pose":{ "position":{x,y,z}, "orientation":{x,y,z,w} },
 *                     "twist":{ "linear":{x,y,z}, "angular":{x,y,z} } }
 */

extern "C" {
#include "node_api.h"
}

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>

#include <string>

#include <nlohmann/json.hpp>

#include "ugv_sdk/details/robot_base/ranger_base.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"
#include "ranger_base/ranger_params.hpp"
#include "ranger_base/kinematics_model.hpp"

namespace {

using json = nlohmann::json;
using namespace westonrobot;

// ============================================================================
// 运行时配置（硬编码，替代 ranger_config.yaml）
// ============================================================================
struct RuntimeConfig {
  std::string can_port = "can0";
  std::string robot_model = "ranger_mini_v3";
  int update_rate = 100;
};

// ============================================================================
// 速度指令（只接收线速度 + 角速度）
// ============================================================================
struct CmdVel {
  float vx = 0.0f;
  float wz = 0.0f;
  bool valid = false;
};

// ============================================================================
// 里程计状态
// ============================================================================
struct OdomState {
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
  bool initialized = false;
  std::chrono::steady_clock::time_point last_update{};
};

// ============================================================================
// 机器人参数
// ============================================================================
struct RobotParams {
  float track;
  float wheelbase;
  float max_linear_speed;
  float max_angular_speed;
  float max_speed_cmd;
  float max_steer_angle_central;
  float max_steer_angle_parallel;
  float max_round_angle;
  float min_turn_radius;
  float max_steer_angle_ackermann;
};

// ============================================================================
// 机器人子型号
// ============================================================================
enum class RangerSubType {
  kRanger = 0,
  kRangerMiniV1,
  kRangerMiniV2,
  kRangerMiniV3
};

// ============================================================================
// 运动模式常量（来自 ranger 协议）
// ============================================================================
namespace MotionMode {
  constexpr uint8_t DUAL_ACKERMAN = 0;
  constexpr uint8_t PARALLEL      = 1;
  constexpr uint8_t SPINNING      = 2;
  constexpr uint8_t SIDE_SLIP     = 3;
}

// ============================================================================
// RangerBringupNode
// ============================================================================
class RangerBringupNode {
public:
  explicit RangerBringupNode(void* dora_context)
      : dora_context_(dora_context) {}

  bool initialize() {
    // 根据机器人型号选择参数和 SDK variant
    if (config_.robot_model == "ranger_mini_v1") {
      robot_type_ = RangerSubType::kRangerMiniV1;
      robot_params_ = {
          RangerMiniV1Params::track, RangerMiniV1Params::wheelbase,
          RangerMiniV1Params::max_linear_speed, RangerMiniV1Params::max_angular_speed,
          RangerMiniV1Params::max_speed_cmd, RangerMiniV1Params::max_steer_angle_central,
          RangerMiniV1Params::max_steer_angle_parallel, RangerMiniV1Params::max_round_angle,
          RangerMiniV1Params::min_turn_radius, RangerMiniV1Params::max_steer_angle_ackermann};
      robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV1);
    } else if (config_.robot_model == "ranger_mini_v2") {
      robot_type_ = RangerSubType::kRangerMiniV2;
      robot_params_ = {
          RangerMiniV2Params::track, RangerMiniV2Params::wheelbase,
          RangerMiniV2Params::max_linear_speed, RangerMiniV2Params::max_angular_speed,
          RangerMiniV2Params::max_speed_cmd, RangerMiniV2Params::max_steer_angle_central,
          RangerMiniV2Params::max_steer_angle_parallel, RangerMiniV2Params::max_round_angle,
          RangerMiniV2Params::min_turn_radius, RangerMiniV2Params::max_steer_angle_ackermann};
      robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV2);
    } else if (config_.robot_model == "ranger_mini_v3") {
      robot_type_ = RangerSubType::kRangerMiniV3;
      robot_params_ = {
          RangerMiniV3Params::track, RangerMiniV3Params::wheelbase,
          RangerMiniV3Params::max_linear_speed, RangerMiniV3Params::max_angular_speed,
          RangerMiniV3Params::max_speed_cmd, RangerMiniV3Params::max_steer_angle_central,
          RangerMiniV3Params::max_steer_angle_parallel, RangerMiniV3Params::max_round_angle,
          RangerMiniV3Params::min_turn_radius, RangerMiniV3Params::max_steer_angle_ackermann};
      robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);
    } else {
      robot_type_ = RangerSubType::kRanger;
      robot_params_ = {
          RangerParams::track, RangerParams::wheelbase,
          RangerParams::max_linear_speed, RangerParams::max_angular_speed,
          RangerParams::max_speed_cmd, RangerParams::max_steer_angle_central,
          RangerParams::max_steer_angle_parallel, RangerParams::max_round_angle,
          RangerParams::min_turn_radius, RangerParams::max_steer_angle_ackermann};
      robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRanger);
    }

    // 连接 CAN 总线
    if (!robot_->Connect(config_.can_port)) {
      std::cerr << "[Ranger] failed to connect CAN port: " << config_.can_port << std::endl;
      return false;
    }
    robot_->EnableCommandedMode();

    std::cout << "[Ranger] connected to CAN: " << config_.can_port
              << ", model: " << config_.robot_model << std::endl;
    return true;
  }

  int run() {
    std::cout << "[Ranger] event loop started" << std::endl;

    while (true) {
      void* event = dora_next_event(dora_context_);
      if (!event) {
        std::cerr << "[Ranger] unexpected end of dora events" << std::endl;
        return -1;
      }

      DoraEventType type = read_dora_event_type(event);

      if (type == DoraEventType_Input) {
        char* data = nullptr;
        size_t data_len = 0;
        char* id = nullptr;
        size_t id_len = 0;

        read_dora_input_data(event, &data, &data_len);
        read_dora_input_id(event, &id, &id_len);

        std::string input_id(id, id_len);

        if (input_id == "timer") {
          /* Test control */
          // printf("[c node] received timer topic\n");
          // sendMotionCommand(0.05,0.00);
          // printf("[c node] ranger node test\n");
          /****************/
          
          handleTimer();

        }
        else if (input_id == "cmd_vel") {
          handleCmdVel(data, data_len);
        }
      }
      else if (type == DoraEventType_Stop) {
        std::cout << "[Ranger] stop event received" << std::endl;
        robot_->SetMotionCommand(0.0, 0.0);
        free_dora_event(event);
        return 0;
      }

      free_dora_event(event);
    }
  }

private:
  // ------------------------------------------------------------------------
  //  定时器：发布里程计
  // ------------------------------------------------------------------------
  void handleTimer() {
    auto state = robot_->GetRobotState();
    auto now = std::chrono::steady_clock::now();

    if (!odom_state_.initialized) {
      odom_state_.initialized = true;
      odom_state_.last_update = now;
      last_time_ = now;
      return;
    }

    float dt = std::chrono::duration<float>(now - odom_state_.last_update).count();
    if (dt <= 0.0f || dt > 1.0f) {
      odom_state_.last_update = now;
      return;
    }
    odom_state_.last_update = now;

    // 从机器人状态获取速度
    float linear_x = state.motion_state.linear_velocity;
    float linear_y = 0.0f;
    float angular_z = state.motion_state.angular_velocity;

    // 根据当前运动模式调整速度分量
    if (motion_mode_ == MotionMode::PARALLEL) {
      float phi = state.motion_state.steering_angle;
      linear_x = state.motion_state.linear_velocity * std::cos(phi);
      linear_y = state.motion_state.linear_velocity * std::sin(phi);
      angular_z = 0.0f;
    } else if (motion_mode_ == MotionMode::SIDE_SLIP) {
      linear_x = 0.0f;
      linear_y = state.motion_state.linear_velocity;
      angular_z = 0.0f;
    }

    // 运动学积分
    updateOdometry(linear_x, angular_z,
                   state.motion_state.steering_angle, dt);

    // 发布里程计
    publishOdometry(linear_x, linear_y, angular_z);
  }

  // ------------------------------------------------------------------------
  //  cmd_vel 处理（只解析 linear.x + angular.z）
  //  格式: { "linear":{"x":v}, "angular":{"z":w} }
  // ------------------------------------------------------------------------
  void handleCmdVel(const char* data, size_t len) {
    CmdVel cmd = parseCmdVel(data, len);
    if (!cmd.valid) return;

    sendMotionCommand(cmd.vx, cmd.wz);
  }

  CmdVel parseCmdVel(const char* data, size_t len) const {
    CmdVel cmd;
    try {
      std::string payload(data, len);
      json j = json::parse(payload);

      cmd.vx = j.value("linear", json::object()).value("x", 0.0f);
      cmd.wz = j.value("angular", json::object()).value("z", 0.0f);

      cmd.valid = std::isfinite(cmd.vx) && std::isfinite(cmd.wz);
    } catch (const std::exception& e) {
      std::cerr << "[Ranger] cmd_vel parse error: " << e.what() << std::endl;
      cmd.valid = false;
    }
    return cmd;
  }

  // ------------------------------------------------------------------------
  //  发送运动指令（线速度 + 角速度 → 阿克曼 / 原地旋转）
  // ------------------------------------------------------------------------
  void sendMotionCommand(float vx, float wz) {
    float radius;
    float steer_cmd = calculateSteeringAngle(vx, wz, radius);

    if (radius < robot_params_.min_turn_radius) {
      // 原地旋转模式
      motion_mode_ = MotionMode::SPINNING;
      robot_->SetMotionMode(MotionMode::SPINNING);

      float av = wz;
      if (av > robot_params_.max_angular_speed) av = robot_params_.max_angular_speed;
      if (av < -robot_params_.max_angular_speed) av = -robot_params_.max_angular_speed;
      robot_->SetMotionCommand(0.0, 0.0, av);
    } else {
      // 双阿克曼转向模式
      motion_mode_ = MotionMode::DUAL_ACKERMAN;
      robot_->SetMotionMode(MotionMode::DUAL_ACKERMAN);

      if (steer_cmd > robot_params_.max_steer_angle_ackermann)
        steer_cmd = robot_params_.max_steer_angle_ackermann;
      if (steer_cmd < -robot_params_.max_steer_angle_ackermann)
        steer_cmd = -robot_params_.max_steer_angle_ackermann;
      robot_->SetMotionCommand(vx, steer_cmd);
    }
  }

  // ------------------------------------------------------------------------
  //  计算阿克曼转向角
  // ------------------------------------------------------------------------
  float calculateSteeringAngle(float vx, float wz, float& radius) const {
    float linear = std::abs(vx);
    float angular = std::abs(wz);

    if (angular < 1e-6f) {
      radius = std::numeric_limits<float>::infinity();
      return 0.0f;
    }

    radius = linear / angular;
    int k = (wz * vx) >= 0 ? 1 : -1;

    float l = robot_params_.wheelbase;
    float phi_i = std::atan((l / 2.0f) / radius);

    const float max_phi_rad = 40.0f * static_cast<float>(M_PI) / 180.0f;
    phi_i = std::min(phi_i, max_phi_rad);

    return static_cast<float>(k) * phi_i;
  }

  // ------------------------------------------------------------------------
  //  运动学积分
  // ------------------------------------------------------------------------
  void updateOdometry(float linear, float angular, float angle, float dt) {
    if (motion_mode_ == MotionMode::DUAL_ACKERMAN) {
      DualAckermanModel::state_type x = {
          static_cast<double>(odom_state_.x),
          static_cast<double>(odom_state_.y),
          static_cast<double>(odom_state_.theta)};
      DualAckermanModel::control_type u;
      u.v = static_cast<double>(linear);
      u.phi = convertInnerAngleToCentral(static_cast<double>(angle));

      boost::numeric::odeint::integrate_const(
          boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
          DualAckermanModel(static_cast<double>(robot_params_.wheelbase), u),
          x, 0.0, static_cast<double>(dt), dt / 10.0);

      odom_state_.x = static_cast<float>(x[0]);
      odom_state_.y = static_cast<float>(x[1]);
      odom_state_.theta = static_cast<float>(x[2]);
    } else if (motion_mode_ == MotionMode::PARALLEL ||
               motion_mode_ == MotionMode::SIDE_SLIP) {
      ParallelModel::state_type x = {
          static_cast<double>(odom_state_.x),
          static_cast<double>(odom_state_.y),
          static_cast<double>(odom_state_.theta)};
      ParallelModel::control_type u;
      u.v = static_cast<double>(linear);
      if (motion_mode_ == MotionMode::SIDE_SLIP) {
        u.phi = M_PI / 2.0;
      } else {
        u.phi = static_cast<double>(angle);
      }
      boost::numeric::odeint::integrate_const(
          boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
          ParallelModel(u), x, 0.0, static_cast<double>(dt), dt / 10.0);

      odom_state_.x = static_cast<float>(x[0]);
      odom_state_.y = static_cast<float>(x[1]);
      odom_state_.theta = static_cast<float>(x[2]);
    } else if (motion_mode_ == MotionMode::SPINNING) {
      SpinningModel::state_type x = {
          static_cast<double>(odom_state_.x),
          static_cast<double>(odom_state_.y),
          static_cast<double>(odom_state_.theta)};
      SpinningModel::control_type u;
      u.w = static_cast<double>(angular);

      boost::numeric::odeint::integrate_const(
          boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
          SpinningModel(u), x, 0.0, static_cast<double>(dt), dt / 10.0);

      odom_state_.x = static_cast<float>(x[0]);
      odom_state_.y = static_cast<float>(x[1]);
      odom_state_.theta = static_cast<float>(x[2]);
    }

    // theta 规范化到 [-π, π]
    while (odom_state_.theta > static_cast<float>(M_PI))
      odom_state_.theta -= 2.0f * static_cast<float>(M_PI);
    while (odom_state_.theta < -static_cast<float>(M_PI))
      odom_state_.theta += 2.0f * static_cast<float>(M_PI);
  }

  // ------------------------------------------------------------------------
  //  内侧转角 → 中央虚拟转角
  // ------------------------------------------------------------------------
  float convertInnerAngleToCentral(float angle) const {
    double phi_i = std::abs(static_cast<double>(angle));
    double phi = std::atan(
        robot_params_.wheelbase * std::sin(phi_i) /
        (robot_params_.wheelbase * std::cos(phi_i) +
         robot_params_.track * std::sin(phi_i)));
    phi *= (angle >= 0.0f) ? 1.0 : -1.0;
    return static_cast<float>(phi);
  }

  // ------------------------------------------------------------------------
  //  发布里程计（简洁格式，参考 dora_mickrobot）
  // ------------------------------------------------------------------------
  void publishOdometry(float vx, float vy, float wz) {
    json j;

    // pose
    j["pose"]["position"]["x"] = odom_state_.x;
    j["pose"]["position"]["y"] = odom_state_.y;
    j["pose"]["position"]["z"] = 0.0f;

    float half_theta = odom_state_.theta * 0.5f;
    j["pose"]["orientation"]["x"] = 0.0f;
    j["pose"]["orientation"]["y"] = 0.0f;
    j["pose"]["orientation"]["z"] = std::sin(half_theta);
    j["pose"]["orientation"]["w"] = std::cos(half_theta);

    // twist
    j["twist"]["linear"]["x"] = vx;
    j["twist"]["linear"]["y"] = vy;
    j["twist"]["linear"]["z"] = 0.0f;

    j["twist"]["angular"]["x"] = 0.0f;
    j["twist"]["angular"]["y"] = 0.0f;
    j["twist"]["angular"]["z"] = wz;

    std::string payload = j.dump();
    const std::string topic = "Odometry";

    int rc = dora_send_output(
        dora_context_,
        const_cast<char*>(topic.c_str()), topic.length(),
        const_cast<char*>(payload.c_str()), payload.length());

    if (rc != 0) {
      std::cerr << "[Ranger] dora_send_output failed (rc=" << rc << ")" << std::endl;
    }
  }

  // ==========================================================================
  //  成员变量
  // ==========================================================================
  void* dora_context_;
  RuntimeConfig config_;

  // 机器人
  std::shared_ptr<RangerRobot> robot_;
  RangerSubType robot_type_ = RangerSubType::kRangerMiniV3;
  RobotParams robot_params_{};
  // 运动模式
  uint8_t motion_mode_ = MotionMode::DUAL_ACKERMAN;

  // 里程计
  OdomState odom_state_;
  std::chrono::steady_clock::time_point last_time_;
};

}  // namespace

// ============================================================================
//  main
// ============================================================================
int main() {
  std::cout << "[Ranger] Dora node starting..." << std::endl;

  void* dora_context = init_dora_context_from_env();
  if (!dora_context) {
    std::cerr << "[Ranger] failed to initialize dora context" << std::endl;
    return -1;
  }

  RangerBringupNode node(dora_context);
  if (!node.initialize()) {
    free_dora_context(dora_context);
    return -1;
  }

  int ret = node.run();

  free_dora_context(dora_context);
  std::cout << "[Ranger] Dora node exited" << std::endl;
  return ret;
}
