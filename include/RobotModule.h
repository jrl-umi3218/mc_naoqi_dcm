#pragma once
#include <sstream>
#include <string>
#include <vector>

namespace mc_naoqi_dcm
{
/** Reimplement std::to_string to support pre-C++11 era */
template<typename T>
inline std::string to_string(T num)
{
  std::stringstream ss;
  ss << num;
  return ss.str();
}

struct JointGroup
{
  std::string groupName;
  std::vector<std::string> jointsNames;
  std::vector<std::string> setActuatorKeys;
  std::vector<std::string> setHardnessKeys;
};

struct rgbLedGroup
{
  std::string groupName;
  std::vector<std::string> ledNames;
  std::vector<std::string> redLedKeys;
  std::vector<std::string> greenLedKeys;
  std::vector<std::string> blueLedKeys;
};

struct iLedGroup
{
  std::string groupName;
  std::vector<std::string> ledNames;
  std::vector<std::string> intensityLedKeys;
};

struct RobotModule
{
  RobotModule();

  // Robot name (e.g. nao or pepper)
  std::string name;
  // Body joints
  std::vector<std::string> actuators;
  // Memory keys of body joints position command
  std::vector<std::string> setActuatorKeys;
  // Memory keys of body joints stiffness command
  std::vector<std::string> setHardnessKeys;
  // All sensors
  std::vector<std::string> sensors;
  // Memory keys to read all sensors with single call to ALMemoryFastAccess
  std::vector<std::string> readSensorKeys;
  // IMU sensor names
  std::vector<std::string> imu;
  // Groups of special robot joints (e.g. wheels)
  std::vector<JointGroup> specialJointGroups;
  // Groups of RGB leds
  std::vector<rgbLedGroup> rgbLedGroups;
  // Groups of single channel leds
  std::vector<iLedGroup> iLedGroups;
  // Bumpers
  std::vector<std::string> bumpers;
  // Tactile sensors
  std::vector<std::string> tactile;

  // Generate memory keys
  void genMemoryKeys(std::string prefix,
                     std::vector<std::string> & devices,
                     std::string postfix,
                     std::vector<std::string> & memory_keys,
                     bool isSensor = false,
                     std::string sensor_prefix = "");
};

} // namespace mc_naoqi_dcm
