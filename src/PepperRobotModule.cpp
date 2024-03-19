#include "PepperRobotModule.h"

namespace mc_naoqi_dcm
{
PepperRobotModule::PepperRobotModule() : RobotModule()
{
  name = "pepper";

  // body joints
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

  // generate memory keys for sending commands to the joints (position/stiffness)
  genMemoryKeys("", actuators, "/Position/Actuator/Value", setActuatorKeys);
  genMemoryKeys("", actuators, "/Hardness/Actuator/Value", setHardnessKeys);

  // generate memory keys for reading sensor values
  // NB! Joint encoders must be in the beginning of the readSensorKeys/sensors
  genMemoryKeys("", actuators, "/Position/Sensor/Value", readSensorKeys, true, "Encoder");
  genMemoryKeys("", actuators, "/ElectricCurrent/Sensor/Value", readSensorKeys, true, "ElectricCurrent");
  genMemoryKeys("InertialSensorBase/", imu, "/Sensor/Value", readSensorKeys, true);

  // wheels - special joint groups
  JointGroup wheels;
  wheels.groupName = "wheels";
  wheels.jointsNames.push_back("WheelFL");
  wheels.jointsNames.push_back("WheelFR");
  wheels.jointsNames.push_back("WheelB");
  genMemoryKeys("", wheels.jointsNames, "/Speed/Actuator/Value", wheels.setActuatorKeys);
  genMemoryKeys("", wheels.jointsNames, "/Stiffness/Actuator/Value", wheels.setHardnessKeys);
  specialJointGroups.push_back(wheels);
  // add sensors for the special joint group
  genMemoryKeys("", wheels.jointsNames, "/Speed/Sensor/Value", readSensorKeys, true, "Encoder");

  // Bumpers
  bumpers.push_back("FrontLeft");
  bumpers.push_back("FrontRight");
  bumpers.push_back("Back");

  genMemoryKeys("Platform/", bumpers, "/Bumper/Sensor/Value", readSensorKeys, true);

  // Tactile sensors
  tactile.push_back("Head/Touch/Front");
  tactile.push_back("Head/Touch/Rear");
  tactile.push_back("Head/Touch/Middle");
  tactile.push_back("RHand/Touch/Back");
  tactile.push_back("LHand/Touch/Back");
  genMemoryKeys("", tactile, "/Sensor/Value", readSensorKeys, true);

  // led groups
  rgbLedGroup eyesCenter;
  eyesCenter.groupName = "eyesCenter";
  eyesCenter.ledNames.push_back("Right/45Deg");
  eyesCenter.ledNames.push_back("Right/225Deg");
  eyesCenter.ledNames.push_back("Left/270Deg");
  eyesCenter.ledNames.push_back("Left/90Deg");
  genMemoryKeys("Face/Led/Red/", eyesCenter.ledNames, "/Actuator/Value", eyesCenter.redLedKeys);
  genMemoryKeys("Face/Led/Green/", eyesCenter.ledNames, "/Actuator/Value", eyesCenter.greenLedKeys);
  genMemoryKeys("Face/Led/Blue/", eyesCenter.ledNames, "/Actuator/Value", eyesCenter.blueLedKeys);
  rgbLedGroups.push_back(eyesCenter);

  rgbLedGroup eyesPeripheral;
  eyesPeripheral.groupName = "eyesPeripheral";
  eyesPeripheral.ledNames.push_back("Right/0Deg");
  eyesPeripheral.ledNames.push_back("Right/90Deg");
  eyesPeripheral.ledNames.push_back("Right/135Deg");
  eyesPeripheral.ledNames.push_back("Right/180Deg");
  eyesPeripheral.ledNames.push_back("Right/270Deg");
  eyesPeripheral.ledNames.push_back("Right/315Deg");
  eyesPeripheral.ledNames.push_back("Left/0Deg");
  eyesPeripheral.ledNames.push_back("Left/45Deg");
  eyesPeripheral.ledNames.push_back("Left/135Deg");
  eyesPeripheral.ledNames.push_back("Left/180Deg");
  eyesPeripheral.ledNames.push_back("Left/225Deg");
  eyesPeripheral.ledNames.push_back("Left/315Deg");
  genMemoryKeys("Face/Led/Red/", eyesPeripheral.ledNames, "/Actuator/Value", eyesPeripheral.redLedKeys);
  genMemoryKeys("Face/Led/Green/", eyesPeripheral.ledNames, "/Actuator/Value", eyesPeripheral.greenLedKeys);
  genMemoryKeys("Face/Led/Blue/", eyesPeripheral.ledNames, "/Actuator/Value", eyesPeripheral.blueLedKeys);
  rgbLedGroups.push_back(eyesPeripheral);

  rgbLedGroup shoulderLeds;
  shoulderLeds.groupName = "shoulderLeds";
  shoulderLeds.ledNames.push_back("");
  genMemoryKeys("ChestBoard/Led/Red", shoulderLeds.ledNames, "/Actuator/Value", shoulderLeds.redLedKeys);
  genMemoryKeys("ChestBoard/Led/Green", shoulderLeds.ledNames, "/Actuator/Value", shoulderLeds.greenLedKeys);
  genMemoryKeys("ChestBoard/Led/Blue", shoulderLeds.ledNames, "/Actuator/Value", shoulderLeds.blueLedKeys);
  rgbLedGroups.push_back(shoulderLeds);

  iLedGroup earsLeds;
  earsLeds.groupName = "earsLeds";
  unsigned numEarLeds = 10;
  unsigned earLedAngleStep = 36;
  // generate memory keys for changing color of all ear leds
  for(unsigned i = 0; i < numEarLeds; i++)
  {
    earsLeds.ledNames.push_back("Right/" + mc_naoqi_dcm::to_string(i * earLedAngleStep) + "Deg");
    earsLeds.ledNames.push_back("Left/" + mc_naoqi_dcm::to_string(i * earLedAngleStep) + "Deg");
  }
  genMemoryKeys("Ears/Led/", earsLeds.ledNames, "/Actuator/Value", earsLeds.intensityLedKeys);
  iLedGroups.push_back(earsLeds);
}

} // namespace mc_naoqi_dcm
