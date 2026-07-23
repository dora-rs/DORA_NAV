/*
 * scout_robot.hpp
 *
 * Created on: Jul 08, 2021 10:59
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef SCOUT_ROBOT_HPP
#define SCOUT_ROBOT_HPP

#include <memory>

#include "ugv_sdk/details/interface/robot_common_interface.hpp"
#include "ugv_sdk/details/interface/tracer_interface.hpp"

namespace westonrobot {
class TracerRobot : public RobotCommonInterface, public TracerInterface {
 public:
  TracerRobot(ProtocolVersion protocol = ProtocolVersion::AGX_V2);
  ~TracerRobot();

  bool Connect(std::string can_name) override;

  void EnableCommandedMode() override;
  std::string RequestVersion(int timeout_sec = 3) override;

  void SetMotionCommand(double linear_vel, double angular_vel) override;
  void SetLightCommand(AgxLightMode f_mode, uint8_t f_value) override;
  void DisableLightControl() override;

  void ResetRobotState() override;

  ProtocolVersion GetParserProtocolVersion() override;

  // get robot state
  TracerCoreState GetRobotState() override;
  TracerActuatorState GetActuatorState() override;
  TracerCommonSensorState GetCommonSensorState() override;

 protected:
  RobotCommonInterface* robot_;
};

}  // namespace westonrobot

#endif /* SCOUT_ROBOT_HPP */
