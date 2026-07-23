/*
 * tracer_base.hpp
 *
 * Created on: Apr 14, 2020 10:21
 * Description:
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#ifndef TRACER_BASE_HPP
#define TRACER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "ugv_sdk/details/interface/tracer_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
template <typename ParserType>
class TracerBase : public AgilexBase<ParserType>,
                     public TracerInterface {
 public:
  TracerBase() : AgilexBase<ParserType>(){};
  ~TracerBase() = default;

  // set up connection
  bool Connect(std::string can_name) override {
    return AgilexBase<ParserType>::Connect(can_name);
  }

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override {
    AgilexBase<ParserType>::SendMotionCommand(linear_vel, angular_vel,
                                                    0.0, 0.0);
  }

  void SetLightCommand(AgxLightMode f_mode, uint8_t f_value) override {
    AgilexBase<ParserType>::SendLightCommand(f_mode, f_value, CONST_OFF,
                                                   0);
  }

  void DisableLightControl() override {
    AgilexBase<ParserType>::DisableLightControl();
  }

  // get robot state
  TracerCoreState GetRobotState() override {
    auto state = AgilexBase<ParserType>::GetRobotCoreStateMsgGroup();

    TracerCoreState tracer_state;
    tracer_state.time_stamp = state.time_stamp;
    tracer_state.system_state = state.system_state;
    tracer_state.motion_state = state.motion_state;
    tracer_state.light_state = state.light_state;
    tracer_state.rc_state = state.rc_state;
    return tracer_state;
  }

  TracerActuatorState GetActuatorState() override {
    auto actuator = AgilexBase<ParserType>::GetActuatorStateMsgGroup();

    TracerActuatorState tracer_actuator;
    tracer_actuator.time_stamp = actuator.time_stamp;
    for (int i = 0; i < 2; ++i) {
      tracer_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
      tracer_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
    }
    return tracer_actuator;
  }

  TracerCommonSensorState GetCommonSensorState() override {
    auto common_sensor =
        AgilexBase<ParserType>::GetCommonSensorStateMsgGroup();

    TracerCommonSensorState tracer_bms;

    tracer_bms.time_stamp = common_sensor.time_stamp;
    tracer_bms.bms_basic_state = common_sensor.bms_basic_state;

    return tracer_bms;
  }


  void ResetRobotState() override {
    // TODO
  }
};
}  // namespace westonrobot

#include "ugv_sdk/details/protocol_v1/protocol_v1_parser.hpp"
#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
using TracerBaseV1 = TracerBase<TracerProtocolV1Parser>;
using TracerBaseV2 = TracerBase<ProtocolV2Parser>;
}  // namespace westonrobot

#endif /* TRACER_BASE_HPP */
