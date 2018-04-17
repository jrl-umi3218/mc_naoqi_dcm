#pragma once
#include <string>
#include <vector>

namespace dcm_module
{
struct RobotModule
{
  std::string name;
  // Name of actuators.
  // mc_rtc RobotModule's ref_joint_order should follow the same ordering.
  std::vector<std::string> actuators;
  // Human readable name for all sensors
  std::vector<std::string> sensors;
  // Human readable name for eye leds
  std::vector<std::string> leds;
  // Human readable name for wheel actuator
  std::vector<std::string> wheels;

  // naoqi device key to set desired actuator values
  std::vector<std::string> setActuatorKeys;
  // naoqi device key to set hardness from memory
  std::vector<std::string> setHardnessKeys;
  // naoqi device key to set led from memory
  std::vector<std::string> setRedLedKeys;
  std::vector<std::string> setGreenLedKeys;
  std::vector<std::string> setBlueLedKeys;
  // naoqi device key to set desired wheel stiffness
  std::vector<std::string> setWheelStiffnessKeys;
  // naoqi device key to set desired wheel speed
  std::vector<std::string> setWheelActuatorKeys;

  // naoqi device key to read sensors from memory
  std::vector<std::string> readSensorKeys;

};

} /* dcm_module */
