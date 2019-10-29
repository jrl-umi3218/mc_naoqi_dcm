#include "NAORobotModule.h"

#include <stdexcept>

namespace dcm_module
{
NAORobotModule::NAORobotModule()
{
  name = "nao";

  // body joints
  actuators = {"HeadPitch", "HeadYaw", "LAnklePitch", "LAnkleRoll",
               "LElbowRoll",  "LElbowYaw","LHand", "LHipPitch", "LHipRoll",
               "LHipYawPitch",  "LKneePitch", "LShoulderPitch", "LShoulderRoll",
               "LWristYaw", "RAnklePitch", "RAnkleRoll", "RElbowRoll", "RElbowYaw",
               "RHand", "RHipPitch", "RHipRoll", "RKneePitch", "RShoulderPitch",
               "RShoulderRoll", "RWristYaw"};

  // generate memory keys for sending commands to the joints (position/stiffness)
  genMemoryKeys("", actuators, "/Position/Actuator/Value", setActuatorKeys);
  genMemoryKeys("", actuators, "/Hardness/Actuator/Value", setHardnessKeys);

  // generate memory keys for reading sensor values
  // NB! Joint encoders must be in the beginning of the readSensorKeys/sensors
  genMemoryKeys("", actuators, "/Position/Sensor/Value", readSensorKeys, true, "Encoder");
  genMemoryKeys("", actuators, "/ElectricCurrent/Sensor/Value", readSensorKeys, true, "ElectricCurrent");
  genMemoryKeys("InertialSensor/", imu, "/Sensor/Value", readSensorKeys, true, "");
  // Force Sensitive Resistors of the feet
  std::vector<std::string> footFSR = {"FrontLeft", "FrontRight", "RearLeft", "RearRight",
                                      "TotalWeight", "CenterOfPressure/X", "CenterOfPressure/Y"};
  genMemoryKeys("LFoot/", footFSR, "/Sensor/Value", readSensorKeys, true, "LFoot");
  genMemoryKeys("RFoot/", footFSR, "/Sensor/Value", readSensorKeys, true, "RFoot");

  // led groups
  rgbLedGroup eyesLeds;
  eyesLeds.groupName = "eyesLeds";
  unsigned numEyeLeds = 8;
  unsigned eyeLedAngleStep = 45;
  // generate memory keys for changing color of all eyes leds
  for (unsigned i = 0; i < numEyeLeds; i++){
    eyesLeds.ledNames.push_back("Right/" + std::to_string(i*eyeLedAngleStep) + "Deg");
    eyesLeds.ledNames.push_back("Left/" + std::to_string(i*eyeLedAngleStep) + "Deg");
  }
  genMemoryKeys("Face/Led/Red/", eyesLeds.ledNames, "/Actuator/Value", eyesLeds.redLedKeys);
  genMemoryKeys("Face/Led/Green/", eyesLeds.ledNames, "/Actuator/Value", eyesLeds.greenLedKeys);
  genMemoryKeys("Face/Led/Blue/", eyesLeds.ledNames, "/Actuator/Value", eyesLeds.blueLedKeys);
  rgbLedGroups.push_back(eyesLeds);

  rgbLedGroup earsLeds;
  earsLeds.groupName = "earsLeds";
  unsigned numEarLeds = 10;
  unsigned earLedAngleStep = 36;
  // generate memory keys for changing color of all ear leds
  for (unsigned i = 0; i < numEarLeds; i++){
    earsLeds.ledNames.push_back("Right/" + std::to_string(i*earLedAngleStep) + "Deg");
    earsLeds.ledNames.push_back("Left/" + std::to_string(i*earLedAngleStep) + "Deg");
  }
  genMemoryKeys("Face/Led/Red/", earsLeds.ledNames, "/Actuator/Value", earsLeds.redLedKeys);
  genMemoryKeys("Face/Led/Green/", earsLeds.ledNames, "/Actuator/Value", earsLeds.greenLedKeys);
  genMemoryKeys("Face/Led/Blue/", earsLeds.ledNames, "/Actuator/Value", earsLeds.blueLedKeys);
  rgbLedGroups.push_back(earsLeds);
}

} /* dcm_module */
