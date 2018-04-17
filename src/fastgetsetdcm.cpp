/// Module to use fast method to get/set joints every 10ms with minimum delays.
/// Implemented for both NAO and PEPPER robots.

#include "fastgetsetdcm.h"
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/alproxy.h>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include "NAORobotModule.h"
#include "PepperRobotModule.h"

#include <alerror/alerror.h>

// Use DCM proxy
#include <alproxies/dcmproxy.h>

// Used to read values of ALMemory directly in RAM
#include <almemoryfastaccess/almemoryfastaccess.h>

#include <alproxies/altexttospeechproxy.h>

#include <boost/bind.hpp>

namespace dcm_module
{
FastGetSetDCM::FastGetSetDCM(boost::shared_ptr<AL::ALBroker> broker,
                             const std::string &name)
    : AL::ALModule(broker, name), fMemoryFastAccess(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess()))
{
  setModuleDescription("Example module to use fast method to get/set joints every 10ms with minimum delays.");

  functionName("startLoop", getName(), "start");
  BIND_METHOD(FastGetSetDCM::startLoop);

  functionName("stopLoop", getName(), "stop");
  BIND_METHOD(FastGetSetDCM::stopLoop);

  functionName("setStiffness", getName(),
               "change stiffness of all joint");
  addParam("value", "new stiffness value from 0.0 to 1.0");
  BIND_METHOD(FastGetSetDCM::setStiffness);

  functionName("setJointAngles", getName(),
               "set joint angles");
  addParam("values", "new joint angles (in radian)");
  BIND_METHOD(FastGetSetDCM::setJointAngles);

  functionName("getJointOrder", getName(), "get reference joint order");
  setReturn("joint order", "array containing joint order");
  BIND_METHOD(FastGetSetDCM::getJointOrder);

  functionName("getSensorsOrder", getName(), "get names for each sensor index");
  setReturn("sensor names", "array containing namess of all the sensors");
  BIND_METHOD(FastGetSetDCM::getSensorsOrder);

  functionName("getSensors", getName(), "get all sensor values");
  setReturn("sensor values", "array containing values of all the sensors");
  BIND_METHOD(FastGetSetDCM::getSensors);

  functionName("sayText", getName(), "Say a given sentence.");
  addParam("toSay", "The sentence to be said.");
  BIND_METHOD(FastGetSetDCM::sayText);

  functionName("changeLedColor", getName(), "changeLedColor");
  addParam("r", "red intensity %");
  addParam("g", "green intensity %");
  addParam("b", "blue intensity %");
  BIND_METHOD(FastGetSetDCM::changeLedColor);

// XXX should be a compile-time check, but no C++11 support makes it tricky
#if defined(PEPPER) || defined(NAO)
#else
#error "Only PEPPER and NAO robots are supported"
#endif

#ifdef PEPPER

  functionName("setWheelsStiffness", getName(),
               "change stiffness of all wheels imediately");
  addParam("value", "new stiffness value from 0.0 to 1.0");
  BIND_METHOD(FastGetSetDCM::setWheelsStiffness);

  functionName("setWheelSpeed", getName(),
               "change wheel speed");
  addParam("speed_fl", "new speed");
  addParam("speed_fr", "new speed");
  addParam("speed_b", "new speed");
  BIND_METHOD(FastGetSetDCM::setWheelSpeed);

  robot_module = PepperRobotModule();
#else
  robot_module = NAORobotModule();
#endif

  startLoop();
}

FastGetSetDCM::~FastGetSetDCM()
{
  stopLoop();
}

void FastGetSetDCM::sayText(const std::string &toSay)
{
  try
  {
    /** Create a proxy to TTS.*/
    AL::ALTextToSpeechProxy tts(getParentBroker());
    /** Call the say method. */
    tts.say(toSay);
    /** Note: on the desktop you won't hear anything, but you should see
    * some logs on the naoqi you are connected to. */
  }
  catch (const AL::ALError &)
  {
    qiLogError("module.example") << "Could not get proxy to ALTextToSpeech" << std::endl;
  }
}

void FastGetSetDCM::changeLedColor(const float &r, const float &g, const float &b)
{
  int DCMtime;
  try
  {
    DCMtime = dcmProxy->getTime(0);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "changeLedColor()", "Error on DCM getTime : " + e.toString());
  }

  redLedCommands[4][0] = DCMtime;

  for (int i = 0; i < robot_module.setRedLedKeys.size(); i++)
  {
    redLedCommands[5][i][0] = r;
  }

  greenLedCommands[4][0] = DCMtime;

  for (int i = 0; i < robot_module.setGreenLedKeys.size(); i++)
  {
    greenLedCommands[5][i][0] = g;
  }

  blueLedCommands[4][0] = DCMtime;

  for (int i = 0; i < robot_module.setBlueLedKeys.size(); i++)
  {
    blueLedCommands[5][i][0] = b;
  }

  try
  {
    dcmProxy->setAlias(redLedCommands);
    dcmProxy->setAlias(greenLedCommands);
    dcmProxy->setAlias(blueLedCommands);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "changeLedColor()", "Error when sending command to DCM : " + e.toString());
  }
}

// Start the example
void FastGetSetDCM::startLoop()
{
  signed long isDCMRunning;

  try
  {
    // Get the DCM proxy
    dcmProxy = getParentBroker()->getDcmProxy();
  }
  catch (AL::ALError &e)
  {
    throw ALERROR(getName(), "startLoop()", "Impossible to create DCM Proxy : " + e.toString());
  }

  // Is the DCM running ?
  try
  {
    isDCMRunning = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));
  }
  catch (AL::ALError &e)
  {
    throw ALERROR(getName(), "startLoop()", "Error when connecting to DCM : " + e.toString());
  }

  if (!isDCMRunning)
  {
    throw ALERROR(getName(), "startLoop()", "Error no DCM running ");
  }

  init();
  connectToDCMloop();
}

void FastGetSetDCM::stopLoop()
{
  setStiffness(0.0f);
  // Remove the postProcess call back connection
  fDCMPostProcessConnection.disconnect();
}

void FastGetSetDCM::init()
{
  initFastAccess();
  createPositionActuatorAlias();
  createHardnessActuatorAlias();
  setStiffness(0.0f);
  preparePositionActuatorCommand();
  createLedAliases();
  prepareLedCommand();

#ifdef PEPPER
  createHardnessWheelAlias();
  createSpeedWheelAlias();
  setWheelsStiffness(0.0f);
  prepareWheelsCommand();
#endif
}

void FastGetSetDCM::initFastAccess()
{
  // Create the fast memory access
  fMemoryFastAccess->ConnectToVariables(getParentBroker(), robot_module.readSensorKeys, false);
}

void FastGetSetDCM::createPositionActuatorAlias()
{
  AL::ALValue jointAliasses;

  jointAliasses.arraySetSize(2);
  jointAliasses[0] = std::string("jointActuator");  // Alias for all actuators position
  jointAliasses[1].arraySetSize(robot_module.setActuatorKeys.size());

  // Joints actuator list
  for (unsigned i = 0; i < robot_module.setActuatorKeys.size(); ++i)
  {
    jointAliasses[1][i] = robot_module.setActuatorKeys[i];
  }

  // Create alias
  try
  {
    dcmProxy->createAlias(jointAliasses);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "createPositionActuatorAlias()", "Error when creating Alias : " + e.toString());
  }
}

void FastGetSetDCM::createLedAlias(std::string aliaseName, std::vector<std::string> keysVector)
{
  AL::ALValue ledAliasses;

  ledAliasses.arraySetSize(2);
  ledAliasses[0] = std::string(aliaseName);
  ledAliasses[1].arraySetSize(keysVector.size());

  // Led list
  for (unsigned i = 0; i < keysVector.size(); ++i)
  {
    ledAliasses[1][i] = keysVector[i];
  }

  // Create alias
  try
  {
    dcmProxy->createAlias(ledAliasses);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "createLedAlias()", "Error when creating Alias : " + e.toString());
  }
}

void FastGetSetDCM::prepareLedCommand()
{
  redLedCommands.arraySetSize(6);
  redLedCommands[0] = std::string("redLed");
  redLedCommands[1] = std::string("ClearAll");
  redLedCommands[2] = std::string("time-separate");
  redLedCommands[3] = 0;
  redLedCommands[4].arraySetSize(1);
  redLedCommands[5].arraySetSize(robot_module.setRedLedKeys.size());

  greenLedCommands.arraySetSize(6);
  greenLedCommands[0] = std::string("greenLed");
  greenLedCommands[1] = std::string("ClearAll");
  greenLedCommands[2] = std::string("time-separate");
  greenLedCommands[3] = 0;
  greenLedCommands[4].arraySetSize(1);
  greenLedCommands[5].arraySetSize(robot_module.setGreenLedKeys.size());

  blueLedCommands.arraySetSize(6);
  blueLedCommands[0] = std::string("blueLed");
  blueLedCommands[1] = std::string("ClearAll");
  blueLedCommands[2] = std::string("time-separate");
  blueLedCommands[3] = 0;
  blueLedCommands[4].arraySetSize(1);
  blueLedCommands[5].arraySetSize(robot_module.setBlueLedKeys.size());

  for (int i = 0; i < robot_module.setRedLedKeys.size(); i++)
  {
    redLedCommands[5][i].arraySetSize(1);
    greenLedCommands[5][i].arraySetSize(1);
    blueLedCommands[5][i].arraySetSize(1);
  }
}

void FastGetSetDCM::createLedAliases()
{
  createLedAlias("redLed", robot_module.setRedLedKeys);
  createLedAlias("greenLed", robot_module.setGreenLedKeys);
  createLedAlias("blueLed", robot_module.setBlueLedKeys);
}

void FastGetSetDCM::createHardnessWheelAlias()
{
  AL::ALValue jointAliasses;
  // Alias for all joint stiffness
  jointAliasses.clear();
  jointAliasses.arraySetSize(2);
  jointAliasses[0] = std::string("wheelStiffness");
  jointAliasses[1].arraySetSize(robot_module.setWheelStiffnessKeys.size());

  // stiffness list
  for (unsigned i = 0; i < robot_module.setWheelStiffnessKeys.size(); ++i)
  {
    jointAliasses[1][i] = robot_module.setWheelStiffnessKeys[i];
  }

  // Create alias
  try
  {
    dcmProxy->createAlias(jointAliasses);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "createHardnessWheelAlias()", "Error when creating Alias : " + e.toString());
  }
}

void FastGetSetDCM::createSpeedWheelAlias()
{
  AL::ALValue jointAliasses;

  jointAliasses.arraySetSize(2);
  jointAliasses[0] = std::string("wheelSpeed");
  jointAliasses[1].arraySetSize(robot_module.setWheelActuatorKeys.size());

  // Joints actuator list
  for (unsigned i = 0; i < robot_module.setWheelActuatorKeys.size(); ++i)
  {
    jointAliasses[1][i] = robot_module.setWheelActuatorKeys[i];
  }

  // Create alias
  try
  {
    dcmProxy->createAlias(jointAliasses);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "createSpeedWheelAlias()", "Error when creating Alias : " + e.toString());
  }
}

void FastGetSetDCM::prepareWheelsCommand()
{
  // prepare command
  wheelCommands.arraySetSize(6);
  wheelCommands[0] = std::string("wheelSpeed");
  wheelCommands[1] = std::string("ClearAll");
  wheelCommands[2] = std::string("time-separate");
  wheelCommands[3] = 0;

  wheelCommands[4].arraySetSize(1);

  wheelCommands[5].arraySetSize(robot_module.setWheelActuatorKeys.size());

  for (int i = 0; i < robot_module.setWheelActuatorKeys.size(); i++)
  {
    wheelCommands[5][i].arraySetSize(1);
  }
}

void FastGetSetDCM::setWheelsStiffness(const float &stiffnessValue)
{
  AL::ALValue stiffnessCommands;
  int DCMtime;
  try
  {
    DCMtime = dcmProxy->getTime(0);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "setStiffness()", "Error on DCM getTime : " + e.toString());
  }

  stiffnessCommands.arraySetSize(3);
  stiffnessCommands[0] = std::string("wheelStiffness");
  stiffnessCommands[1] = std::string("Merge");
  stiffnessCommands[2].arraySetSize(1);
  stiffnessCommands[2][0].arraySetSize(2);
  stiffnessCommands[2][0][0] = stiffnessValue;
  stiffnessCommands[2][0][1] = DCMtime;
  try
  {
    dcmProxy->set(stiffnessCommands);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "setWheelsStiffness()", "Error when sending stiffness to DCM : " + e.toString());
  }
}

void FastGetSetDCM::setWheelSpeed(const float &speed_fl, const float &speed_fr, const float &speed_b)
{
  int DCMtime;

  try
  {
    DCMtime = dcmProxy->getTime(0);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "setWheelSpeed()", "Error on DCM getTime : " + e.toString());
  }

  wheelCommands[4][0] = DCMtime;
  wheelCommands[5][0][0] = speed_fl;
  wheelCommands[5][1][0] = speed_fr;
  wheelCommands[5][2][0] = speed_b;

  try
  {
    dcmProxy->setAlias(wheelCommands);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "setWheelSpeed()", "Error when sending command to DCM : " + e.toString());
  }
}

void FastGetSetDCM::createHardnessActuatorAlias()
{
  AL::ALValue jointAliasses;
  // Alias for all joint stiffness
  jointAliasses.clear();
  jointAliasses.arraySetSize(2);
  jointAliasses[0] = std::string("jointStiffness");  // Alias for all actuators stiffness
  jointAliasses[1].arraySetSize(robot_module.setHardnessKeys.size());

  // stiffness list
  for (unsigned i = 0; i < robot_module.setHardnessKeys.size(); ++i)
  {
    jointAliasses[1][i] = robot_module.setHardnessKeys[i];
  }

  // Create alias
  try
  {
    dcmProxy->createAlias(jointAliasses);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "createHardnessActuatorAlias()", "Error when creating Alias : " + e.toString());
  }
}

void FastGetSetDCM::preparePositionActuatorCommand()
{
  commands.arraySetSize(6);
  commands[0] = std::string("jointActuator");
  commands[1] = std::string("ClearAll");  // Erase all previous commands
  commands[2] = std::string("time-separate");
  commands[3] = 0;

  commands[4].arraySetSize(1);
  //commands[4][0]  Will be the new time

  commands[5].arraySetSize(robot_module.setActuatorKeys.size());  // For all joints
  for (unsigned i = 0; i < robot_module.setActuatorKeys.size(); i++)
  {
    commands[5][i].arraySetSize(1);
    //commands[5][i][0] will be the new angle
  }
}

void FastGetSetDCM::setStiffness(const float &stiffnessValue)
{
  AL::ALValue stiffnessCommands;
  int DCMtime;
  // increase stiffness with the "jointStiffness" Alias created at initialisation
  try
  {
    DCMtime = dcmProxy->getTime(0);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "setStiffness()", "Error on DCM getTime : " + e.toString());
  }

  // Prepare one dcm command:
  // it will linearly "Merge" all joint stiffness
  // from last value to "stiffnessValue" in 1 seconde
  stiffnessCommands.arraySetSize(3);
  stiffnessCommands[0] = std::string("jointStiffness");
  stiffnessCommands[1] = std::string("ClearAll");
  stiffnessCommands[2].arraySetSize(1);
  stiffnessCommands[2][0].arraySetSize(2);
  stiffnessCommands[2][0][0] = stiffnessValue;
  stiffnessCommands[2][0][1] = DCMtime;
  try
  {
    dcmProxy->set(stiffnessCommands);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "setStiffness()", "Error when sending stiffness to DCM : " + e.toString());
  }
}

void FastGetSetDCM::setJointAngles(std::vector<float> jointValues)
{
  initialJointSensorValues = jointValues;
}

std::vector<std::string> FastGetSetDCM::getJointOrder() const
{
  return robot_module.actuators;
}

std::vector<std::string> FastGetSetDCM::getSensorsOrder() const
{
  return robot_module.sensors;
}

std::vector<float> FastGetSetDCM::getSensors()
{
  // Get all values from ALMemory using fastaccess
  fMemoryFastAccess->GetValues(sensorValues);
  return sensorValues;
}

void FastGetSetDCM::connectToDCMloop()
{
  // Get all values from ALMemory using fastaccess
  fMemoryFastAccess->GetValues(sensorValues);

  // Save all sensor position for the sensor=actuator test
  for (int i = 0; i < 17; i++)
  {
    initialJointSensorValues.push_back(sensorValues[i]);
  }

  // Connect callback to the DCM post proccess
  try
  {
    //  onPreProcess is useful because it’s called just before the computation of orders sent to the chestboard (USB). Sending commands at this level means that you have the shortest delay to your command.
    fDCMPostProcessConnection =
        getParentBroker()->getProxy("DCM")->getModule()->atPreProcess(boost::bind(&FastGetSetDCM::synchronisedDCMcallback, this));
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "connectToDCMloop()", "Error when connecting to DCM postProccess: " + e.toString());
  }
}

void FastGetSetDCM::synchronisedDCMcallback()
{
  int DCMtime;

  try
  {
    // Get absolute time, at 0 ms in the future ( i.e. now )
    DCMtime = dcmProxy->getTime(0);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "synchronisedDCMcallback()", "Error on DCM getTime : " + e.toString());
  }

  commands[4][0] = DCMtime;  // To be used in the next cycle

  // XXX make this faster with memcpy?
  for (unsigned i = 0; i < robot_module.actuators.size(); i++)
  {
    // new actuator value = first Sensor value
    commands[5][i][0] = initialJointSensorValues[i];
  }

  try
  {
    dcmProxy->setAlias(commands);
  }
  catch (const AL::ALError &e)
  {
    throw ALERROR(getName(), "synchronisedDCMcallback()", "Error when sending command to DCM : " + e.toString());
  }
}

} /* dcm_module */
