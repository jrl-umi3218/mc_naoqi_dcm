#include "PepperRobotModule.h"

namespace dcm_module
{
PepperRobotModule::PepperRobotModule() : RobotModule()
{
  name = "pepper";
  actuators.push_back("KneePitch");
  actuators.push_back("HipPitch");
  actuators.push_back("HipRoll");
  actuators.push_back("HeadYaw");
  actuators.push_back("HeadPitch");
  actuators.push_back("LShoulderPitch");
  actuators.push_back("LShoulderRoll");
  actuators.push_back("LElbowYaw");
  actuators.push_back("LElbowRoll");
  actuators.push_back("LWristYaw");
  actuators.push_back("LHand");
  actuators.push_back("RShoulderPitch");
  actuators.push_back("RShoulderRoll");
  actuators.push_back("RElbowYaw");
  actuators.push_back("RElbowRoll");
  actuators.push_back("RWristYaw");
  actuators.push_back("RHand");

  for (unsigned i = 0; i < actuators.size(); ++i)
  {
    const std::string& actuator = actuators[i];
    sensors.push_back("Encoder" + actuator);
    setActuatorKeys.push_back("Device/SubDeviceList/" + actuator + "/Position/Actuator/Value");
    setHardnessKeys.push_back("Device/SubDeviceList/" + actuator + "/Hardness/Actuator/Value");
    readSensorKeys.push_back("Device/SubDeviceList/" + actuator + "/Position/Sensor/Value");
  }

  sensors.push_back("AccelerometerX");
  sensors.push_back("AccelerometerY");
  sensors.push_back("AccelerometerZ");
  sensors.push_back("GyroscopeX");
  sensors.push_back("GyroscopeY");
  sensors.push_back("GyroscopeZ");
  sensors.push_back("AngleX");
  sensors.push_back("AngleY");
  sensors.push_back("AngleZ");

  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");
}

} /* dcm_module */
