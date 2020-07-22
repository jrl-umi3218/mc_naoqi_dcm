#include "PepperRobotModule.h"

namespace mc_naoqi_dcm
{
PepperRobotModule::PepperRobotModule() : RobotModule()
{
  name = "pepper";

  // body joints
  actuators = {"KneePitch", "HipPitch", "HipRoll", "HeadYaw", "HeadPitch",
   	"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
   	"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"};

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
  wheels.jointsNames = {"WheelFL", "WheelFR", "WheelB"};
  genMemoryKeys("", wheels.jointsNames, "/Speed/Actuator/Value", wheels.setActuatorKeys);
  genMemoryKeys("", wheels.jointsNames, "/Stiffness/Actuator/Value", wheels.setHardnessKeys);
  specialJointGroups.push_back(wheels);
  // add sensors for the special joint group
  genMemoryKeys("", wheels.jointsNames, "/Speed/Sensor/Value", readSensorKeys, true, "Encoder");

  // Bumpers
  bumpers = {"FrontLeft", "FrontRight", "Back"};
  genMemoryKeys("Platform/", bumpers, "/Bumper/Sensor/Value", readSensorKeys, true);

  // led groups
  rgbLedGroup eyesCenter;
  eyesCenter.groupName = "eyesCenter";
  eyesCenter.ledNames = {"Right/45Deg", "Right/225Deg", "Left/270Deg", "Left/90Deg"};
  genMemoryKeys("Face/Led/Red/", eyesCenter.ledNames, "/Actuator/Value", eyesCenter.redLedKeys);
  genMemoryKeys("Face/Led/Green/", eyesCenter.ledNames, "/Actuator/Value", eyesCenter.greenLedKeys);
  genMemoryKeys("Face/Led/Blue/", eyesCenter.ledNames, "/Actuator/Value", eyesCenter.blueLedKeys);
  rgbLedGroups.push_back(eyesCenter);

  rgbLedGroup eyesPeripheral;
  eyesPeripheral.groupName = "eyesPeripheral";
  eyesPeripheral.ledNames = {"Right/0Deg", "Right/90Deg", "Right/135Deg", "Right/180Deg", "Right/270Deg", "Right/315Deg",
                             "Left/0Deg", "Left/45Deg", "Left/135Deg", "Left/180Deg", "Left/225Deg", "Left/315Deg"};
  genMemoryKeys("Face/Led/Red/", eyesPeripheral.ledNames, "/Actuator/Value", eyesPeripheral.redLedKeys);
  genMemoryKeys("Face/Led/Green/", eyesPeripheral.ledNames, "/Actuator/Value", eyesPeripheral.greenLedKeys);
  genMemoryKeys("Face/Led/Blue/", eyesPeripheral.ledNames, "/Actuator/Value", eyesPeripheral.blueLedKeys);
  rgbLedGroups.push_back(eyesPeripheral);

  rgbLedGroup shoulderLeds;
  shoulderLeds.groupName = "shoulderLeds";
  shoulderLeds.ledNames = {""};
  genMemoryKeys("ChestBoard/Led/Red", shoulderLeds.ledNames, "/Actuator/Value", shoulderLeds.redLedKeys);
  genMemoryKeys("ChestBoard/Led/Green", shoulderLeds.ledNames, "/Actuator/Value", shoulderLeds.greenLedKeys);
  genMemoryKeys("ChestBoard/Led/Blue", shoulderLeds.ledNames, "/Actuator/Value", shoulderLeds.blueLedKeys);
  rgbLedGroups.push_back(shoulderLeds);

  iLedGroup earsLeds;
  earsLeds.groupName = "earsLeds";
  unsigned numEarLeds = 10;
  unsigned earLedAngleStep = 36;
  // generate memory keys for changing color of all ear leds
  for (unsigned i = 0; i < numEarLeds; i++){
    earsLeds.ledNames.push_back("Right/" + std::to_string(i*earLedAngleStep) + "Deg");
    earsLeds.ledNames.push_back("Left/" + std::to_string(i*earLedAngleStep) + "Deg");
  }
  genMemoryKeys("Ears/Led/", earsLeds.ledNames, "/Actuator/Value", earsLeds.intensityLedKeys);
  iLedGroups.push_back(earsLeds);
}

} /* mc_naoqi_dcm */
