/// Module to communicate with mc_rtc_naoqi interface for whole-body control via mc_rtc framework
/// Enables connection of a preproccess callback to DCM loop for sending joint commands every 12ms
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
FastGetSetDCM::FastGetSetDCM(boost::shared_ptr<AL::ALBroker> broker, const std::string &name)
    : AL::ALModule(broker, name), fMemoryFastAccess(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess()))
{
  setModuleDescription("Module to communicate with mc_rtc_naoqi interface for whole-body control via mc_rtc framework");

  // Bind methods to make them accessible through proxies
  functionName("startLoop", getName(), "connect a callback to DCM loop");
  BIND_METHOD(FastGetSetDCM::startLoop);

  functionName("stopLoop", getName(), "disconnect a callback from DCM loop");
  BIND_METHOD(FastGetSetDCM::stopLoop);

  functionName("isPreProccessConnected", getName(), "Check if preProccess is connected");
  setReturn("if connected", "boolean indicating if preProccess is connected to DCM loop");
  BIND_METHOD(FastGetSetDCM::isPreProccessConnected);

  functionName("setStiffness", getName(), "change stiffness of all joint");
  addParam("value", "new stiffness value from 0.0 to 1.0");
  BIND_METHOD(FastGetSetDCM::setStiffness);

  functionName("setJointAngles", getName(), "set joint angles");
  addParam("values", "new joint angles (in radian)");
  BIND_METHOD(FastGetSetDCM::setJointAngles);

  functionName("getJointOrder", getName(), "get reference joint order");
  setReturn("joint order", "array containing names of all the joints");
  BIND_METHOD(FastGetSetDCM::getJointOrder);

  functionName("getSensorsOrder", getName(), "get reference sensor order");
  setReturn("sensor names", "array containing names of all the sensors");
  BIND_METHOD(FastGetSetDCM::getSensorsOrder);

  functionName("numSensors", getName(), "get total number of sensors");
  setReturn("sensors number", "int indicating total number of different sensors");
  BIND_METHOD(FastGetSetDCM::numSensors);

  functionName("getSensors", getName(), "get all sensor values");
  setReturn("sensor values", "array containing values of all the sensors");
  BIND_METHOD(FastGetSetDCM::getSensors);

  functionName("sayText", getName(), "Say a given sentence.");
  addParam("toSay", "The sentence to be said.");
  BIND_METHOD(FastGetSetDCM::sayText);

  functionName("setLeds", getName(), "setLeds");
  addParam("ledGroupName", "Name of the leds group from robot module");
  addParam("r", "red intensity %");
  addParam("g", "green intensity %");
  addParam("b", "blue intensity %");
  addParam("delay", "delay in ms");
  BIND_METHOD(FastGetSetDCM::setLeds);

  functionName("isetLeds", getName(), "isetLeds");
  addParam("ledGroupName", "Name of the leds group from robot module");
  addParam("b", "blue intensity %");
  BIND_METHOD(FastGetSetDCM::isetLeds);

  functionName("blink", getName(), "blink");
  BIND_METHOD(FastGetSetDCM::blink);

  #ifdef PEPPER
    // Bind methods specific to Pepper robot
    functionName("setWheelsStiffness", getName(), "change wheels stiffness");
    addParam("value", "new stiffness value from 0.0 to 1.0");
    BIND_METHOD(FastGetSetDCM::setWheelsStiffness);

    functionName("setWheelSpeed", getName(), "change wheel speed");
    addParam("speed_fl", "front left wheel speed");
    addParam("speed_fr", "front right wheel speed");
    addParam("speed_b", "back wheel speed");
    BIND_METHOD(FastGetSetDCM::setWheelSpeed);

    // Create Pepper robot module
    robot_module = PepperRobotModule();
  #else
    // Create NAO robot module
    robot_module = NAORobotModule();
  #endif

  // Get the DCM proxy
  try{
    dcmProxy = getParentBroker()->getDcmProxy();
  }catch (AL::ALError &e){
    throw ALERROR(getName(), "FastGetSetDCM", "Impossible to create DCM Proxy : " + e.toString());
  }

  // Check that DCM is running
  signed long isDCMRunning;
  try{
    isDCMRunning = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));
  }catch (AL::ALError &e){
    throw ALERROR(getName(), "FastGetSetDCM", "Error when connecting to DCM : " + e.toString());
  }

  if (!isDCMRunning){
    throw ALERROR(getName(), "FastGetSetDCM", "Error no DCM running ");
  }

  // initialize sensor reading/setting
  init();

  // Get all sensor values from ALMemory using fastaccess
  fMemoryFastAccess->GetValues(sensorValues);

  // Save initial sensor values into 'jointPositionCommands'
  for (int i = 0; i < robot_module.actuators.size(); i++){
    jointPositionCommands.push_back(sensorValues[i]);
  }

  // Send initial command to the actuators
  int DCMtime;
  try{
    // Get absolute time, at 0 ms in the future ( i.e. now )
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "FastGetSetDCM", "Error on DCM getTime : " + e.toString());
  }
  commands[4][0] = DCMtime;
  for (unsigned i = 0; i < robot_module.actuators.size(); i++){
    commands[5][i][0] = jointPositionCommands[i];
  }
  try{
    dcmProxy->setAlias(commands);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "FastGetSetDCM", "Error when sending command to DCM : " + e.toString());
  }
}

// Module destructor
FastGetSetDCM::~FastGetSetDCM()
{
  setWheelSpeed(0.0f, 0.0f, 0.0f);
  setStiffness(0.0f);
  setWheelsStiffness(0.0f);
  stopLoop();
}

// Start loop
void FastGetSetDCM::startLoop()
{
  connectToDCMloop();
  preProcessConnected = true;
}

// Stop loop
void FastGetSetDCM::stopLoop()
{
  // Remove the preProcess callback connection
  fDCMPreProcessConnection.disconnect();
  preProcessConnected = false;
}

bool FastGetSetDCM::isPreProccessConnected(){
  return preProcessConnected;
}

void FastGetSetDCM::init()
{
  // Enable fast access of all robot_module.readSensorKeys from memory
  initFastAccess();
  // create 'jointActuator' alias to be used for sending joint possition commands
  createAliasPrepareCommand("jointActuator", robot_module.setActuatorKeys, commands);
  // create 'jointStiffness' alias to be used for setting joint stiffness commands
  createAliasPrepareCommand("jointStiffness", robot_module.setHardnessKeys, jointStiffnessCommands);
  // keep body joints turned off at initialization
  setStiffness(0.0f);
  // prepare commands for all led groups of robot_module
  createLedAliases();

  #ifdef PEPPER
    // create wheels speed and stiffness commands
    const JointGroup& wheels = robot_module.specialJointGroups[0];
    std::string wheelsSpeedAliasName = wheels.groupName+std::string("Speed");
    std::string wheelsStiffnessAliasName = wheels.groupName+std::string("Stiffness");
    createAliasPrepareCommand(wheelsSpeedAliasName, wheels.setActuatorKeys, wheelsCommands);
    createAliasPrepareCommand(wheelsStiffnessAliasName, wheels.setHardnessKeys, wheelsStiffnessCommands);
    // keep wheels turned off at initialization
    setWheelsStiffness(0.0f);
  #endif
}

void FastGetSetDCM::initFastAccess(){
  // Create the fast memory access to read sensor values
  fMemoryFastAccess->ConnectToVariables(getParentBroker(), robot_module.readSensorKeys, false);
}


void FastGetSetDCM::createAliasPrepareCommand(std::string aliasName,
                const std::vector<std::string> &mem_keys,
                AL::ALValue& alias_command, std::string updateType){

  // create alias (unite group of memory keys under specific alis name)
  AL::ALValue alias;
  // 2 - for alias name and array of alias memory keys
  alias.arraySetSize(2);
  alias[0] = std::string(aliasName);
  alias[1].arraySetSize(mem_keys.size());

  // fill in array of memory keys of the alias
  for (unsigned i = 0; i < mem_keys.size(); ++i){
    alias[1][i] = mem_keys[i];
  }

  // Create alias in DCM
  try{
    dcmProxy->createAlias(alias);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "createLedAlias()", "Error when creating Alias : " + e.toString());
  }

  // alias created succesfully
  // prepare command for this alias to be set via DCM 'setAlias' call
  alias_command.arraySetSize(6);
  alias_command[0] = std::string(aliasName);
  alias_command[1] = std::string(updateType);
  alias_command[2] = std::string("time-separate");
  alias_command[3] = 0; // Importance level. Not yet implemented. Must be set to 0
  // placeholder for command time
  alias_command[4].arraySetSize(1);
  // placeholder for command values
  alias_command[5].arraySetSize(mem_keys.size());
  for (int i = 0; i < mem_keys.size(); i++){
    // allocate space for a new value for a memory key to be set via setAlias call
    alias_command[5][i].arraySetSize(1);
  }
}

void FastGetSetDCM::createLedAliases()
{
  // RGB led groups
  for(int i=0;i<robot_module.rgbLedGroups.size();i++){
    const rgbLedGroup& leds = robot_module.rgbLedGroups[i];
    AL::ALValue redLedCommands;
    AL::ALValue greenLedCommands;
    AL::ALValue blueLedCommands;
    std::string rName = leds.groupName+std::string("Red");
    std::string gName = leds.groupName+std::string("Green");
    std::string bName = leds.groupName+std::string("Blue");
    createAliasPrepareCommand(rName, leds.redLedKeys, redLedCommands, "Merge");
    createAliasPrepareCommand(gName, leds.greenLedKeys, greenLedCommands, "Merge");
    createAliasPrepareCommand(bName, leds.blueLedKeys, blueLedCommands, "Merge");
    // map led group name to led commands
    ledCmdMap[leds.groupName] = {redLedCommands, greenLedCommands, blueLedCommands};
  }

  // Single channel led groups
  for(int i=0;i<robot_module.iLedGroups.size();i++){
    const iLedGroup& leds = robot_module.iLedGroups[i];
    AL::ALValue intensityLedCommands;
    createAliasPrepareCommand(leds.groupName, leds.intensityLedKeys, intensityLedCommands, "Merge");
    // map led group name to led commands
    ledCmdMap[leds.groupName] = {intensityLedCommands};
  }
}

void FastGetSetDCM::setWheelsStiffness(const float &stiffnessValue)
{
  int DCMtime;
  try{
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "setStiffness()", "Error on DCM getTime : " + e.toString());
  }

  wheelsStiffnessCommands[4][0] = DCMtime;
  wheelsStiffnessCommands[5][0][0] = stiffnessValue;
  wheelsStiffnessCommands[5][1][0] = stiffnessValue;
  wheelsStiffnessCommands[5][2][0] = stiffnessValue;

  try  {
    dcmProxy->setAlias(wheelsStiffnessCommands);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "setWheelsStiffness()", "Error when sending stiffness to DCM : " + e.toString());
  }
}

void FastGetSetDCM::setWheelSpeed(const float &speed_fl, const float &speed_fr, const float &speed_b)
{
  int DCMtime;
  try{
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e)  {
    throw ALERROR(getName(), "setWheelSpeed()", "Error on DCM getTime : " + e.toString());
  }

  wheelsCommands[4][0] = DCMtime;
  wheelsCommands[5][0][0] = speed_fl;
  wheelsCommands[5][1][0] = speed_fr;
  wheelsCommands[5][2][0] = speed_b;

  try{
    dcmProxy->setAlias(wheelsCommands);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "setWheelSpeed()", "Error when sending command to DCM : " + e.toString());
  }
}

void FastGetSetDCM::setStiffness(const float &stiffnessValue)
{
  int DCMtime;
  // increase stiffness with the "jointStiffness" Alias created at initialisation
  try{
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "setStiffness()", "Error on DCM getTime : " + e.toString());
  }

  jointStiffnessCommands[4][0] = DCMtime;

  for(int i=0;i<robot_module.actuators.size();i++){
    jointStiffnessCommands[5][i][0] = stiffnessValue;
  }

  try{
    dcmProxy->setAlias(jointStiffnessCommands);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "setStiffness()", "Error when sending stiffness to DCM : " + e.toString());
  }
}

void FastGetSetDCM::setJointAngles(std::vector<float> jointValues)
{
  // update values in the vector that is used to send joint commands every 12ms
  jointPositionCommands = jointValues;
}

std::vector<std::string> FastGetSetDCM::getJointOrder() const
{
  return robot_module.actuators;
}

std::vector<std::string> FastGetSetDCM::getSensorsOrder() const
{
  return robot_module.sensors;
}

int FastGetSetDCM::numSensors() const
{
  return robot_module.readSensorKeys.size();
}

std::vector<float> FastGetSetDCM::getSensors()
{
  // Get all values from ALMemory using fastaccess
  fMemoryFastAccess->GetValues(sensorValues);
  return sensorValues;
}

void FastGetSetDCM::connectToDCMloop()
{
  // Connect callback to the DCM pre proccess
  try{
    //  onPreProcess is useful because it’s called just before the computation of orders sent to the chestboard (USB). Sending commands at this level means that you have the shortest delay to your command.
    fDCMPreProcessConnection =
        getParentBroker()->getProxy("DCM")->getModule()->atPreProcess(boost::bind(&FastGetSetDCM::synchronisedDCMcallback, this));
        // what happens if I add it twice? add same callback while it is already added? try in python?
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "connectToDCMloop()", "Error when connecting to DCM preProccess: " + e.toString());
  }
}

// using 'jointActuator' alias created 'command'
// that will use data from 'jointPositionCommands' to send it to DCM every 12 ms
void FastGetSetDCM::synchronisedDCMcallback()
{
  int DCMtime;

  try{
    // Get absolute time, at 0 ms in the future ( i.e. now )
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "synchronisedDCMcallback()", "Error on DCM getTime : " + e.toString());
  }

  commands[4][0] = DCMtime;

  // XXX make this faster with memcpy?
  for (unsigned i = 0; i < robot_module.actuators.size(); i++){
    // new actuator value = latest values from jointPositionCommands
    commands[5][i][0] = jointPositionCommands[i];
  }

  try{
    dcmProxy->setAlias(commands);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "synchronisedDCMcallback()", "Error when sending command to DCM : " + e.toString());
  }
}


void FastGetSetDCM::sayText(const std::string &toSay)
{
  try{
    AL::ALTextToSpeechProxy tts(getParentBroker());
    tts.say(toSay);
  }catch (const AL::ALError &){
    qiLogError("module.example") << "Could not get proxy to ALTextToSpeech" << std::endl;
  }
}


void FastGetSetDCM::setLeds(std::string ledGroupName, const float &r, const float &g, const float &b)
{
  int DCMtime;
  try{
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "changeShouldersLeds()", "Error on DCM getTime : " + e.toString());
  }

  // TODO: proceed only if such led group exists!
  std::vector<AL::ALValue> &rgbCmnds = ledCmdMap[ledGroupName];

  qiLogInfo("rgbCmnds[0].getSize()") << rgbCmnds[0].getSize() << std::endl;
  qiLogInfo("rgbCmnds[0][5].getSize()") << rgbCmnds[0][5].getSize() << std::endl;

  rgbCmnds[0][4][0] = DCMtime;
  rgbCmnds[1][4][0] = DCMtime;
  rgbCmnds[2][4][0] = DCMtime;

  // set RGB values for every memory key of this led group
  for (int i = 0; i < rgbCmnds[0][5].getSize(); i++){
    rgbCmnds[0][5][i][0] = r;
    rgbCmnds[1][5][i][0] = g;
    rgbCmnds[2][5][i][0] = b;
  }

  try{
    dcmProxy->setAlias(rgbCmnds[0]);
    dcmProxy->setAlias(rgbCmnds[1]);
    dcmProxy->setAlias(rgbCmnds[2]);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "changeShouldersLeds()", "Error when sending command to DCM : " + e.toString());
  }
}

void FastGetSetDCM::setLedsDelay(std::string ledGroupName, const float &r, const float &g, const float &b, const int& delay)
{
  int DCMtime;
  try{
    DCMtime = dcmProxy->getTime(delay);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "changeShouldersLeds()", "Error on DCM getTime : " + e.toString());
  }

  // TODO: proceed only if such led group exists!
  std::vector<AL::ALValue> &rgbCmnds = ledCmdMap[ledGroupName];

  qiLogInfo("rgbCmnds[0].getSize()") << rgbCmnds[0].getSize() << std::endl;
  qiLogInfo("rgbCmnds[0][5].getSize()") << rgbCmnds[0][5].getSize() << std::endl;

  rgbCmnds[0][4][0] = DCMtime;
  rgbCmnds[1][4][0] = DCMtime;
  rgbCmnds[2][4][0] = DCMtime;

  // set RGB values for every memory key of this led group
  for (int i = 0; i < rgbCmnds[0][5].getSize(); i++){
    rgbCmnds[0][5][i][0] = r;
    rgbCmnds[1][5][i][0] = g;
    rgbCmnds[2][5][i][0] = b;
  }

  try{
    dcmProxy->setAlias(rgbCmnds[0]);
    dcmProxy->setAlias(rgbCmnds[1]);
    dcmProxy->setAlias(rgbCmnds[2]);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "changeShouldersLeds()", "Error when sending command to DCM : " + e.toString());
  }
}

void FastGetSetDCM::isetLeds(std::string ledGroupName, const float &intensity)
{
  int DCMtime;
  try{
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "changeShouldersLeds()", "Error on DCM getTime : " + e.toString());
  }

  std::vector<AL::ALValue> &intensityCmnds = ledCmdMap[ledGroupName];
  // assert if intensityCmnds.size()==1, indeed single channel led group

  intensityCmnds[0][4][0] = DCMtime;

  // set intensity values for every memory key of this led group
  for (int i = 0; i < intensityCmnds[0][5].getSize(); i++){
    intensityCmnds[0][5][i][0] = intensity;
  }

  try{
    dcmProxy->setAlias(intensityCmnds[0]);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "changeShouldersLeds()", "Error when sending command to DCM : " + e.toString());
  }
}

void FastGetSetDCM::blink()
{
  // This is possible because led aliases update type is "Merge"
  setLedsDelay("eyesPeripheral", 0.0, 0.0, 0.0, 75);
  setLedsDelay("eyesPeripheral", 0.0, 0.0, 0.0, 225);
  setLedsDelay("eyesPeripheral", 1.0, 1.0, 1.0, 300);
  setLedsDelay("eyesCenter", 0.0, 0.0, 0.0, 150);
  setLedsDelay("eyesCenter", 1.0, 1.0, 1.0, 300);
}

} /* dcm_module */
