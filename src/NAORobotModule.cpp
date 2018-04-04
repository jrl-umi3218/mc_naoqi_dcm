#include "NAORobotModule.h"

#include <stdexcept>

namespace dcm_module
{
NAORobotModule::NAORobotModule()
{
  name = "nao";

  // Joints Sensor list
  actuators.push_back("HeadPitch");
  actuators.push_back("HeadYaw");
  actuators.push_back("LAnklePitch");
  actuators.push_back("LAnkleRoll");
  actuators.push_back("LElbowRoll");
  actuators.push_back("LElbowYaw");
  actuators.push_back("LHand");
  actuators.push_back("LHipPitch");
  actuators.push_back("LHipRoll");
  actuators.push_back("LHipYawPitch");
  actuators.push_back("LKneePitch");
  actuators.push_back("LShoulderPitch");
  actuators.push_back("LShoulderRoll");
  actuators.push_back("LWristYaw");
  actuators.push_back("RAnklePitch");
  actuators.push_back("RAnkleRoll");
  actuators.push_back("RElbowRoll");
  actuators.push_back("RElbowYaw");
  actuators.push_back("RHand");
  actuators.push_back("RHipPitch");
  actuators.push_back("RHipRoll");
  actuators.push_back("RKneePitch");
  actuators.push_back("RShoulderPitch");
  actuators.push_back("RShoulderRoll");
  actuators.push_back("RWristYaw");

  for (unsigned i = 0; i < actuators.size(); ++i)
  {
    const std::string& actuator = actuators[i];
    sensors.push_back("Encoder" + actuator);
    setActuatorKeys.push_back("Device/SubDeviceList/" + actuator + "/Position/Actuator/Value");
    setHardnessKeys.push_back("Device/SubDeviceList/" + actuator + "/Hardness/Actuator/Value");
    readSensorKeys.push_back("Device/SubDeviceList/" + actuator + "/Position/Sensor/Value");
  }

  // Inertial sensors
  sensors.push_back("AccelerometerX");
  sensors.push_back("AccelerometerY");
  sensors.push_back("AccelerometerZ");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");

  sensors.push_back("GyroscopeX");
  sensors.push_back("GyroscopeY");
  sensors.push_back("GyroscopeZ");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value");

  sensors.push_back("AngleX");
  sensors.push_back("AngleY");
  sensors.push_back("AngleZ");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");

  sensors.push_back("LF_FSR_FrontLeft");
  sensors.push_back("LF_FSR_FrontRight");
  sensors.push_back("LF_FSR_RearLeft");
  sensors.push_back("LF_FSR_RearRight");
  sensors.push_back("RF_FSR_FrontLeft");
  sensors.push_back("RF_FSR_FrontRight");
  sensors.push_back("RF_FSR_RearLeft");
  sensors.push_back("RF_FSR_RearRight");
  sensors.push_back("LF_FSR_TotalWeight");
  sensors.push_back("RF_FSR_TotalWeight");
  readSensorKeys.push_back("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value");

  // Some FSR sensors
  sensors.push_back("LF_FSR_COP_X");
  sensors.push_back("LF_FSR_COP_Y");
  sensors.push_back("RF_FSR_COP_X");
  sensors.push_back("RF_FSR_COP_Y");
  readSensorKeys.push_back("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/X/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/Y/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/X/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/Y/Sensor/Value");
}

} /* dcm_module */
