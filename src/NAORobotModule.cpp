#include "NAORobotModule.h"

#include <stdexcept>

namespace dcm_module
{
NAORobotModule::NAORobotModule()
{
  name = "nao";

  // Joints Sensor list
  actuators.push_back("Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LHand/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/LWristYaw/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RHand/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
  actuators.push_back("Device/SubDeviceList/RWristYaw/Position/Sensor/Value");

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
