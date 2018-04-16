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

  /*! Create DCM hardness Actuator Alias */
  void createHardnessActuatorAlias();

  /*! Create DCM Position Actuator Alias */
  void createPositionActuatorAlias();

  /*! Prepare Command ALValue to send command to actuator */
  void preparePositionActuatorCommand();

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
  void sayText(const std::string& toSay);

  /**
   * Create DCM Eye Led Alias
   */
  void createLedAlias(std::string aliaseName, std::vector<std::string> keysVector);
  void createLedAliases();

  /*! Prepare Command ALValue to send command to led */
  void prepareLedCommand();

  /**
   * Change eye color
   */
  void changeLedColor(const float &r, const float &g, const float &b);

  /**
   * Store command to send to leds
   */
  AL::ALValue redLedCommands;
  AL::ALValue greenLedCommands;
  AL::ALValue blueLedCommands;

  /*! Create DCM hardness wheel alias */
  void createHardnessWheelAlias();

  /*! Create DCM śpeed wheel alias */
  void createSpeedWheelAlias();

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

  /**
   * Store commands to send to wheels
   */
  AL::ALValue wheelCommands;

  /*! Prepare Command ALValue to send command to wheels */
  void prepareWheelsCommand();

 private:
  // Used for postprocess sync with the DCM
  ProcessSignalConnection fDCMPostProcessConnection;

  // Used for fast memory access
  boost::shared_ptr<AL::ALMemoryFastAccess> fMemoryFastAccess;

  // Store sensor values.
  std::vector<float> sensorValues;
  boost::shared_ptr<AL::DCMProxy> dcmProxy;

  // Used for the test actuator = sensor
  std::vector<float> initialJointSensorValues;

  // Used to store command to send
  AL::ALValue commands;

  RobotModule robot_module;
};

} /* dcm_module */
