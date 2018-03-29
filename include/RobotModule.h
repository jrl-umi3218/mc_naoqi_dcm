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

  // naoqi device key to set desired actuator values
  std::vector<std::string> setActuatorKeys;
  // naoqi device key to set hardness from memory
  std::vector<std::string> setHardnessKeys;

  // naoqi device key to read sensors from memory
  std::vector<std::string> readSensorKeys;
};

} /* dcm_module */
