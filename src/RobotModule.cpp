#include "RobotModule.h"

namespace mc_naoqi_dcm
{

RobotModule::RobotModule()
{
  imu.push_back("AccelerometerX");
  imu.push_back("AccelerometerY");
  imu.push_back("AccelerometerZ");
  imu.push_back("GyroscopeX");
  imu.push_back("GyroscopeY");
  imu.push_back("GyroscopeZ");
  imu.push_back("AngleX");
  imu.push_back("AngleY");
  imu.push_back("AngleZ");
}

// Function to create various memory key groups
void RobotModule::genMemoryKeys(std::string prefix,
                                std::vector<std::string> & devices,
                                std::string postfix,
                                std::vector<std::string> & memory_keys,
                                bool isSensor,
                                std::string sensor_prefix)
{
  // Create memory keys for all devices
  for(unsigned i = 0; i < devices.size(); i++)
  {
    memory_keys.push_back("Device/SubDeviceList/" + prefix + devices[i] + postfix);
    // If device is a sensor, also add it to human-readable sensors vector
    if(isSensor)
    {
      sensors.push_back(sensor_prefix + devices[i]);
    }
  }
}

} // namespace mc_naoqi_dcm
