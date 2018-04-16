#include "PepperRobotModule.h"

namespace dcm_module
{
PepperRobotModule::PepperRobotModule() : RobotModule()
{
  name = "pepper";

  // Joints Sensor list
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

  // Inertial sensors
  sensors.push_back("AccelerometerX");
  sensors.push_back("AccelerometerY");
  sensors.push_back("AccelerometerZ");
  sensors.push_back("GyroscopeX");
  sensors.push_back("GyroscopeY");
  sensors.push_back("GyroscopeZ");
  sensors.push_back("AngleX");
  sensors.push_back("AngleY");
  sensors.push_back("AngleZ");

  readSensorKeys.push_back("Device/SubDeviceList/InertialSensorBase/AccelerometerX/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensorBase/AccelerometerY/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensorBase/AccelerometerZ/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensorBase/GyroscopeX/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensorBase/GyroscopeY/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensorBase/GyroscopeZ/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensorBase/AngleX/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensorBase/AngleY/Sensor/Value");
  readSensorKeys.push_back("Device/SubDeviceList/InertialSensorBase/AngleZ/Sensor/Value");

  // Leds
  leds.push_back("Left/45Deg");
  leds.push_back("Left/90Deg");
  leds.push_back("Right/0Deg");
  leds.push_back("Right/45Deg");

  for (unsigned i = 0; i < leds.size(); ++i)
  {
    setRedLedKeys.push_back("Device/SubDeviceList/Face/Led/Red/" + leds[i] + "/Actuator/Value");
    setGreenLedKeys.push_back("Device/SubDeviceList/Face/Led/Green/" + leds[i] + "/Actuator/Value");
    setBlueLedKeys.push_back("Device/SubDeviceList/Face/Led/Blue/" + leds[i] + "/Actuator/Value");
  }

  // Wheels
  wheels.push_back("WheelFL");
  wheels.push_back("WheelFR");
  wheels.push_back("WheelB");

  for (unsigned i = 0; i < wheels.size(); ++i)
  {
    setWheelStiffnessKeys.push_back("Device/SubDeviceList/" + wheels[i] + "/Stiffness/Actuator/Value");
    setWheelActuatorKeys.push_back("Device/SubDeviceList/" + wheels[i] + "/Speed/Actuator/Value");
  }
}

} /* dcm_module */
