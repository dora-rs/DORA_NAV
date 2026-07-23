extern "C" {
#include "node_api.h"
}

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <serial/serial.h>

namespace {

using json = nlohmann::json;


struct RuntimeConfig {
  std::string serial_port = "/dev/ttyUSB0";
  uint32_t baudrate = 115200;
  uint32_t serial_timeout_ms = 5;
  std::string cmd_vel_input_id = "cmd_vel";
  std::string odom_output_id = "Odometry";
  std::chrono::milliseconds cmd_vel_timeout{500};
  float speed_offset_mps = 10.0f;
  float speed_scale = 100.0f;
  float min_linear_deadband = 0.01f;
  float min_angular_deadband = 0.001f;
};

struct CmdVel {
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
  bool valid = false;
};

struct ChassisState {
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
  bool has_feedback = false;
};

struct OdomState {
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
  bool initialized = false;
  std::chrono::steady_clock::time_point last_update{};
};

class MickX4BringupNode {
public:
  explicit MickX4BringupNode(void* dora_context)
      : dora_context_(dora_context),
        last_cmd_time_(std::chrono::steady_clock::now()) {}

  bool initialize() {
    try {
      serial_.setPort(config_.serial_port);
      serial_.setBaudrate(config_.baudrate);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      timeout.inter_byte_timeout = 1;
      timeout.read_timeout_constant = config_.serial_timeout_ms;
      timeout.read_timeout_multiplier = 0;
      serial_.setTimeout(timeout);
      serial_.open();
      serial_.flushInput();
    } catch (const serial::IOException&) {
      std::cerr << "Unable to open serial port: " << config_.serial_port << std::endl;
      return false;
    }

    if (!serial_.isOpen()) {
      std::cerr << "Serial open failed: " << config_.serial_port << std::endl;
      return false;
    }

    std::cout << "mickx4_bringup started. serial=" << config_.serial_port
              << " baud=" << config_.baudrate << std::endl;
    return true;
  }

  int run() {
    while (true) {
      void* event = dora_next_event(dora_context_); /* 阻塞等待 */
      if (event == nullptr) {
        std::cerr << "Unexpected end of dora events" << std::endl;
        return -1;
      }
      
      DoraEventType type = read_dora_event_type(event);
      if (type == DoraEventType_Input) {
        handleInputEvent(event);
      } else if (type == DoraEventType_Stop) {
        std::cout << "Received stop event" << std::endl;
        sendChassisSpeed(0.0f, 0.0f, 0.0f);
        free_dora_event(event);
        return 0;
      }

      free_dora_event(event);
    }
  }

private:
  static int16_t beToInt16(uint8_t high, uint8_t low) {
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low));
  }

  static bool isFinite3(float a, float b, float c) {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
  }

  void handleInputEvent(void* event) {
    char* data = nullptr;
    size_t data_len = 0;
    char* id = nullptr;
    size_t id_len = 0;

    read_dora_input_data(event, &data, &data_len);
    read_dora_input_id(event, &id, &id_len);

    const std::string input_id(id, id_len);
    if (input_id == "timer") {
      pollSerialAndPublishOdometry();
      enforceCmdVelTimeout();
    }
    else if (input_id == config_.cmd_vel_input_id) {
      CmdVel cmd = parseCmdVel(data, data_len);
      if (!cmd.valid) {
        return;
      }
      sendChassisSpeed(cmd.vx, cmd.vy, cmd.wz);
      last_cmd_time_ = std::chrono::steady_clock::now();
      sent_timeout_stop_ = false;
    }

  }

  CmdVel parseCmdVel(const char* data, size_t len) const {
    CmdVel cmd;
    try {
      std::string payload(data, len);
      json j = json::parse(payload);

      cmd.vx = j.value("linear", json::object()).value("x", 0.0f);
      cmd.vy = 0.0f;
      cmd.wz = j.value("angular", json::object()).value("z", 0.0f);

      cmd.valid = isFinite3(cmd.vx, cmd.vy, cmd.wz);
    } catch (const std::exception& e) {
      std::cerr << "[cmd_vel] parse failed: " << e.what() << std::endl;
      cmd.valid = false;
    }

    return cmd;
  }

  std::vector<uint8_t> buildSpeedFrame(float vx, float vy, float wz) const {
    std::vector<uint8_t> frame;
    frame.reserve(16);

    auto encodeSpeed = [&](float v) {
      const int scaled = static_cast<int>((v + config_.speed_offset_mps) * config_.speed_scale);
      const uint16_t u = static_cast<uint16_t>(scaled);
      frame.push_back(static_cast<uint8_t>(u / 256));
      frame.push_back(static_cast<uint8_t>(u % 256));
    };

    frame.push_back(0xAE);
    frame.push_back(0xEA);
    frame.push_back(0x0B);   // length placeholder, back-filled below
    frame.push_back(0xF3);

    encodeSpeed(vx);
    encodeSpeed(vy);
    encodeSpeed(wz);

    frame.push_back(0x00);
    frame.push_back(0x00);

    uint32_t checksum = 0;
    for (size_t i = 2; i < frame.size(); ++i) {
      checksum += frame[i];
    }
    frame.push_back(static_cast<uint8_t>(checksum & 0xFF));

    frame[2] = static_cast<uint8_t>(frame.size() - 2);

    frame.push_back(0xEF);
    frame.push_back(0xFE);
    return frame;
  }

  bool sendChassisSpeed(float vx, float vy, float wz) {
    if (!serial_.isOpen()) {
      std::cerr << "[serial] cannot send speed: port not open" << std::endl;
      return false;
    }

    std::vector<uint8_t> frame = buildSpeedFrame(vx, vy, wz);
    size_t written = serial_.write(frame);
    if (written != frame.size()) {
      std::cerr << "[serial] write incomplete: " << written << "/" << frame.size() << " bytes" << std::endl;
      return false;
    }
    return true;
  }

  void pollSerialAndPublishOdometry() {
    if (!serial_.isOpen()) {
      return;
    }

    size_t avail = serial_.available();
    if (avail == 0) {
      return;
    }

    std::string chunk = serial_.read(avail);
    rx_buffer_.insert(rx_buffer_.end(), chunk.begin(), chunk.end());
    parseFeedbackFrames();

    if (chassis_state_.has_feedback) {
      updateOdometryAndPublish();
      chassis_state_.has_feedback = false;
    }
  }

  void parseFeedbackFrames() {
    size_t pos = 0;
    size_t dropped = 0;

    while (rx_buffer_.size() >= pos + 6) {
      // Scan for frame header
      while (rx_buffer_.size() >= pos + 2 &&
             !(static_cast<uint8_t>(rx_buffer_[pos])     == 0xAE &&
               static_cast<uint8_t>(rx_buffer_[pos + 1]) == 0xEA)) {
        ++pos;
        ++dropped;
      }

      if (rx_buffer_.size() < pos + 4) {
        break;
      }

      const uint8_t len_field = static_cast<uint8_t>(rx_buffer_[pos + 2]);
      const size_t frame_len = static_cast<size_t>(len_field) + 4;

      if (rx_buffer_.size() < pos + frame_len) {
        break;
      }

      const size_t end = pos + frame_len;
      if (static_cast<uint8_t>(rx_buffer_[end - 2]) != 0xEF ||
          static_cast<uint8_t>(rx_buffer_[end - 1]) != 0xFE) {
        // std::cerr << "[serial] frame tail mismatch, skipping" << std::endl;
        ++pos;
        ++dropped;
        continue;
      }

      parseOneFrame(reinterpret_cast<const uint8_t*>(&rx_buffer_[pos]), frame_len);
      pos = end;
    }

    if (dropped > 0) {
      std::cerr << "[serial] discarded " << dropped << " unframed byte(s)" << std::endl;
    }

    if (pos > 0) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(pos));
    }
  }

  void parseOneFrame(const uint8_t* frame, size_t frame_len) {
    if (frame_len < 12) {
      // std::cerr << "[parse] frame too short: " << frame_len << " bytes" << std::endl;
      return;
    }

    const uint8_t frame_type = frame[3];
    if (frame_type != 0xA7) {
      // std::cerr << "[parse] unknown frame type: 0x" << std::hex
      //           << static_cast<unsigned>(frame_type) << std::dec << std::endl;
      return;
    }

    size_t i = 4;
    if (i + 5 >= frame_len) {
      std::cerr << "[parse] feedback payload truncated" << std::endl;
      return;
    }

    int16_t vx_raw = beToInt16(frame[i], frame[i + 1]);
    i += 2;
    int16_t vy_raw = beToInt16(frame[i], frame[i + 1]);
    i += 2;
    int16_t wz_raw = beToInt16(frame[i], frame[i + 1]);

    chassis_state_.vx = static_cast<float>(vx_raw) / 1000.0f;
    chassis_state_.vy = static_cast<float>(vy_raw) / 1000.0f;
    chassis_state_.wz = static_cast<float>(wz_raw) / 1000.0f;
    chassis_state_.has_feedback = true;

  }

  void updateOdometryAndPublish() {
    const auto now = std::chrono::steady_clock::now();

    if (!odom_state_.initialized) {
      odom_state_.initialized = true;
      odom_state_.last_update = now;
      return;
    }

    double dt = std::chrono::duration<double>(now - odom_state_.last_update).count();
    odom_state_.last_update = now;

    if (dt <= 0.0 || dt > 1.0) {
      std::cerr << "[odom] skipping integration: dt=" << dt << "s" << std::endl;
      return;
    }

    float vx = chassis_state_.vx;
    float vy = chassis_state_.vy;
    float wz = chassis_state_.wz;

    if (std::fabs(vx) < config_.min_linear_deadband) {
      vx = 0.0f;
    }
    if (std::fabs(wz) < config_.min_angular_deadband) {
      wz = 0.0f;
    }

    odom_state_.x += std::cos(odom_state_.theta) * vx * static_cast<float>(dt);
    odom_state_.y += std::sin(odom_state_.theta) * vx * static_cast<float>(dt);
    odom_state_.theta += wz * static_cast<float>(dt);

    publishOdometry(vx, vy, wz);
  }

  void publishOdometry(float vx, float vy, float wz) {
    json j;
    j["pose"]["position"]["x"] = odom_state_.x;
    j["pose"]["position"]["y"] = odom_state_.y;
    j["pose"]["position"]["z"] = 0.0;

    j["pose"]["orientation"]["x"] = 0.0;
    j["pose"]["orientation"]["y"] = 0.0;
    j["pose"]["orientation"]["z"] = 0.0;
    j["pose"]["orientation"]["w"] = 1.0;

    j["twist"]["linear"]["x"] = vx;
    j["twist"]["linear"]["y"] = vy;
    j["twist"]["linear"]["z"] = 0.0;

    j["twist"]["angular"]["x"] = 0.0;
    j["twist"]["angular"]["y"] = 0.0;
    j["twist"]["angular"]["z"] = wz;

    std::string payload = j.dump();
    int rc = dora_send_output(dora_context_,
                              const_cast<char*>(config_.odom_output_id.c_str()),
                              config_.odom_output_id.length(),
                              const_cast<char*>(payload.c_str()),
                              payload.length());

    if (rc != 0) {
      std::cerr << "[odom] dora_send_output failed, rc=" << rc << std::endl;
    }
  }

  void enforceCmdVelTimeout() {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_cmd_time_ < config_.cmd_vel_timeout) {
      return;
    }

    if (!sent_timeout_stop_) {
      std::cerr << "[safety] cmd_vel timeout, stopping chassis" << std::endl;
      sendChassisSpeed(0.0f, 0.0f, 0.0f);
      sent_timeout_stop_ = true;
    }
  }

private:
  void* dora_context_;
  RuntimeConfig config_;
  serial::Serial serial_;

  std::vector<char> rx_buffer_;
  ChassisState chassis_state_;
  OdomState odom_state_;

  std::chrono::steady_clock::time_point last_cmd_time_;
  bool sent_timeout_stop_ = false;
};

}  // namespace

int main() {
  std::cout << "MickrobotX4 bringup Dora node" << std::endl;

  void* dora_context = init_dora_context_from_env();
  if (dora_context == nullptr) {
    std::cerr << "Failed to initialize Dora context" << std::endl;
    return -1;
  }

  MickX4BringupNode node(dora_context);
  if (!node.initialize()) {
    free_dora_context(dora_context);
    return -1;
  }

  int ret = node.run();
  free_dora_context(dora_context);

  std::cout << "MickrobotX4 node exited" << std::endl;
  return ret;
}
