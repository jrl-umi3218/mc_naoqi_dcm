#include "NAORobotModule.h"

#include <stdexcept>

namespace mc_naoqi_dcm
{

NAORobotModule::NAORobotModule()
{
  name = "nao";

  // body joints
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

  // generate memory keys for sending commands to the joints (position/stiffness)
  genMemoryKeys("", actuators, "/Position/Actuator/Value", setActuatorKeys);
  genMemoryKeys("", actuators, "/Hardness/Actuator/Value", setHardnessKeys);

  // generate memory keys for reading sensor values
  // NB! Joint encoders must be in the beginning of the readSensorKeys/sensors
  genMemoryKeys("", actuators, "/Position/Sensor/Value", readSensorKeys, true, "Encoder");
  genMemoryKeys("", actuators, "/ElectricCurrent/Sensor/Value", readSensorKeys, true, "ElectricCurrent");
  genMemoryKeys("InertialSensor/", imu, "/Sensor/Value", readSensorKeys, true, "");
  // Force Sensitive Resistors of the feet
  std::vector<std::string> footFSR;
  footFSR.push_back("FrontLeft");
  footFSR.push_back("FrontRight");
  footFSR.push_back("RearLeft");
  footFSR.push_back("RearRight");
  footFSR.push_back("TotalWeight");
  footFSR.push_back("CenterOfPressure/X");
  footFSR.push_back("CenterOfPressure/Y");

  genMemoryKeys("LFoot/", footFSR, "/Sensor/Value", readSensorKeys, true, "LFoot");
  genMemoryKeys("RFoot/", footFSR, "/Sensor/Value", readSensorKeys, true, "RFoot");

  // Feet bumpers (two per each foot)
  bumpers.push_back("LFoot");
  bumpers.push_back("RFoot");
  genMemoryKeys("", bumpers, "/Bumper/Right/Sensor/Value", readSensorKeys, true);
  genMemoryKeys("", bumpers, "/Bumper/Left/Sensor/Value", readSensorKeys, true);

  // led groups
  rgbLedGroup eyesLeds;
  eyesLeds.groupName = "eyesLeds";
  unsigned numEyeLeds = 8;
  unsigned eyeLedAngleStep = 45;
  // generate memory keys for changing color of all eyes leds
  for(unsigned i = 0; i < numEyeLeds; i++)
  {
    eyesLeds.ledNames.push_back("Right/" + mc_naoqi_dcm::to_string(i * eyeLedAngleStep) + "Deg");
    eyesLeds.ledNames.push_back("Left/" + mc_naoqi_dcm::to_string(i * eyeLedAngleStep) + "Deg");
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
  for(unsigned i = 0; i < numEarLeds; i++)
  {
    earsLeds.ledNames.push_back("Right/" + mc_naoqi_dcm::to_string(i * earLedAngleStep) + "Deg");
    earsLeds.ledNames.push_back("Left/" + mc_naoqi_dcm::to_string(i * earLedAngleStep) + "Deg");
  }
  genMemoryKeys("Face/Led/Red/", earsLeds.ledNames, "/Actuator/Value", earsLeds.redLedKeys);
  genMemoryKeys("Face/Led/Green/", earsLeds.ledNames, "/Actuator/Value", earsLeds.greenLedKeys);
  genMemoryKeys("Face/Led/Blue/", earsLeds.ledNames, "/Actuator/Value", earsLeds.blueLedKeys);
  rgbLedGroups.push_back(earsLeds);
}

} // namespace mc_naoqi_dcm
