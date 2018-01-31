/// <summary>
/// Example module to use fast method to get/set joints every 10ms with minimum delays.
/// </summary>

#ifndef _FAST_GET_SET_DCM_H
#define _FAST_GET_SET_DCM_H

#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>

namespace AL
{
class ALBroker;
class ALMemoryFastAccess;
class DCMProxy;
}

/// <summary>
/// Example module tu use fast method to get/set joints every 10ms with minimum delays.
/// </summary>
class FastGetSetDCM : public AL::ALModule
{
 public:
  FastGetSetDCM(boost::shared_ptr<AL::ALBroker> pBroker,
                const std::string &pName);

  virtual ~FastGetSetDCM();

  // Start the example
  void startLoop();

  // Stop the example
  void stopLoop();

 private:
  // Initialisation of ALMemory/DCM link
  void init();

  // ALMemory fast access
  void initFastAccess();

  //  Connect callback to the DCM post proccess
  void connectToDCMloop();

  // Callback called by the DCM every 10ms
  void synchronisedDCMcallback();

  // Create DCM hardness Actuator Alias
  void createHardnessActuatorAlias();

  // Create DCM Position Actuator Alias
  void createPositionActuatorAlias();

  // Prepare Command ALValue to send command to actuator
  void preparePositionActuatorCommand();

  // Set one hardness value to all joint
  void setStiffness(const float &stiffnessValue);
  void setJointAngles(std::vector<float> jointValues);
  AL::ALValue getJointOrder() const;
  std::vector<std::string> getSensorsOrder() const;
  std::vector<float> getSensors();

  // Used for postprocess sync with the DCM
  ProcessSignalConnection fDCMPostProcessConnection;

  // Sensors names
  std::vector<std::string> fSensorKeys;
  std::vector<std::string> fActuatorKeys;

  // Used for fast memory access
  boost::shared_ptr<AL::ALMemoryFastAccess> fMemoryFastAccess;

  // Store sensor values.
  std::vector<float> sensorValues;
  boost::shared_ptr<AL::DCMProxy> dcmProxy;

  // Used for the test actuator = sensor
  std::vector<float> initialJointSensorValues;

  // Used to store command to send
  AL::ALValue commands;
};

enum SensorType
{
  KNEE_PITCH,
  HIP_PITCH,
  HIP_ROLL,
  HEAD_YAW,
  HEAD_PITCH,
  L_SHOULDER_PITCH,
  L_SHOULDER_ROLL,
  L_ELBOW_YAW,
  L_ELBOW_ROLL,
  L_WRIST_YAW,
  L_HAND,
  R_SHOULDER_PITCH,
  R_SHOULDER_ROLL,
  R_ELBOW_YAW,
  R_ELBOW_ROLL,
  R_WRIST_YAW,
  R_HAND,

  ACC_X,
  ACC_Y,
  ACC_Z,
  GYR_X,
  GYR_Y,
  GYR_Z,
  ANGLE_X,
  ANGLE_Y,
  ANGLE_Z,

  SENSOR_SIZE
};

#endif  // _FAST_GET_SET_DCM_H
