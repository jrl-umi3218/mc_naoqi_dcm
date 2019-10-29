#pragma once
#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include "RobotModule.h"

namespace AL
{
class ALBroker;
class ALMemoryFastAccess;
class DCMProxy;
}

namespace dcm_module
{
/**
 * @brief Module to use fast method to get/set joints every 10ms with minimum delays.
 * Supported robots are PEPPER and NAO, but it should be suitable to for
 * extension to other Softbank Robotics robots.
 */
class FastGetSetDCM : public AL::ALModule
{
 public:
  /**
   * @brief Module to use fast method to get/set joints every 10ms with minimum delays.
   *
   * @param broker A smart pointer to the broker (communication object)
   * @param name The name of the module
   */
  FastGetSetDCM(boost::shared_ptr<AL::ALBroker> pBroker,
                const std::string &pName);

  virtual ~FastGetSetDCM();

  /*! Start the example */
  void startLoop();

  /*! Stop the example */
  void stopLoop();

 private:
  /*! Initialisation of ALMemory/DCM link */
  void init();

  /*! ALMemory fast access */
  void initFastAccess();

  /*!  Connect callback to the DCM post proccess */
  void connectToDCMloop();

  /**
  * @brief Callback called by the DCM every 10ms
  *
  *  Once this method is connected to DCM postprocess
  *  it will be called in Real Time every 10 milliseconds from DCM thread
  *  Dynamic allocation and system call are strictly forbidden in this method
  *  Computation time in this section must remain as short as possible to prevent
  *  erratic move or joint getting loose.
  *
  */
  void synchronisedDCMcallback();

  /**
   * @brief Set one hardness value to all joint
   *
   * @param stiffnessValue
   * Stiffness value that will be applied to all joints
   */
  void setStiffness(const float &stiffnessValue);

  /**
   * @brief Sets the desired actuator position to the specified one.
   *
   * @param jointValues
   * Joint values, specified in the same order as RobotModule::actuators.
   */
  void setJointAngles(std::vector<float> jointValues);

  /**
   * @brief Joint order in which the actuator values will be expressed
   *
   * @return
   * Reference joint order as defined in RobotModule::actuators
   */
  std::vector<std::string> getJointOrder() const;

  /**
   * @brief List of readable sensor names.
   * getSensors() will return sensor values corresponding to these.
   *
   * @return
   * Human-readable list of sensor names, as defined in RobotModule::sensors
   */
  std::vector<std::string> getSensorsOrder() const;

  /**
   * @brief Sensor values in the order expressed by getSensorsOrder()
   *
   * @return Vector of sensor values
   */
  std::vector<float> getSensors();

  /**
   * Make robot say a sentence given in argument
   */
  void sayText(const std::string &toSay);

  /**
   * Create DCM alias and prepare command (ALValue structure) for it
   */
  void createAliasPrepareCommand(std::string aliasName,
                                 const std::vector<std::string> &mem_keys,
                                 AL::ALValue& ledCommands,
                                 std::string updateType="ClearAll");
  // Create aliases for all leg groups defined in robot module
  // TODO: probably rename this function
  void createLedAliases();

  /**
   * @brief Set one hardness value to all wheels
   *
   * @param stiffnessValue
   * Stiffness value that will be applied to all wheels
   */
  void setWheelsStiffness(const float &stiffnessValue);

  /**
   * @brief Set speed values to the wheels
   *
   * @param speed_fl
   * Speed value that will be applied to front left wheel
   * @param speed_fr
   * Speed value that will be applied to front right wheel
   * @param speed_b
   * Speed value that will be applied to back wheel
   */
  void setWheelSpeed(const float &speed_fl, const float &speed_fr, const float &speed_b);

  // one led set function for all groups
  void setLeds(std::string ledGroupName, const float &r, const float &g, const float &b);
  void setLedsDelay(std::string ledGroupName, const float &r, const float &g, const float &b, const int& delay);
  // TODO: better implementation/naming for similar-functionality methods
  // cannot bind mathods with the same name and different arguments
  void isetLeds(std::string ledGroupName, const float &intensity);
  // blink
  void blink();

 private:
  // Used for postprocess sync with the DCM
  ProcessSignalConnection fDCMPostProcessConnection;

  // Used for fast memory access
  boost::shared_ptr<AL::ALMemoryFastAccess> fMemoryFastAccess;

  // Store sensor values.
  std::vector<float> sensorValues;
  boost::shared_ptr<AL::DCMProxy> dcmProxy;

  // Used for sending joint position commands every 10ms in callback
  // TODO: it probably makes sense to rename this variable to better reflect it's function
  std::vector<float> initialJointSensorValues;

  // Used to store joint possition command to set via DCM every 12ms
  AL::ALValue commands;

  // joint stiffness command for DCM
  AL::ALValue jointStiffnessCommands;

  /**
   * Store command to send to leds
   */

  // map led group name to corresponding RGB or intensity commands
  std::map<std::string, std::vector<AL::ALValue>> ledCmdMap;

  /**
   * Store commands to send to wheels (speed and stiffness)
   */
  AL::ALValue wheelsCommands;
  AL::ALValue wheelsStiffnessCommands;

  /**
   * \brief The RobotModule describes the sensors names and their corresponding
   * naoqi keys. The intent is to have a generic dcm module for both NAO and
   * PEPPER robots.
   */
  RobotModule robot_module;

  /**
   * Total number of sensors to be read from the memeory
   * Allows to pre-set apropriate vector size for storing and updating all sensor readings
   */
  int numSensors() const;
};

} /* dcm_module */
