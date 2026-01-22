/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: Long Vu
 * @LastEditTime: 2026-01-12 20:18:14
 */

#pragma once

// cpp
#include <atomic>
#include <memory>
#include <string>

// projects
#include "common_type.h"

namespace xyber {

class JointController {
 public:
  // Singletons
  JointController(JointController& other) = delete;
  void operator=(const JointController&) = delete;
  virtual ~JointController();

  /**
   * @brief: Get or create JointController instance
   * @return {*}
   */
  static JointController* GetInstance();

  /**
   * @brief Get the Version object
   *
   * @return std::string, "main.sub.patch"
   */
  std::string GetVersion();

  /**
   * @brief: Create a spi device instance in JointController
   * @param {string} spi device name
   * @param {uint8_t} spi bus
   * @param {uint8_t} spi chip select
   * @return {bool} true for success
   */
  bool CreateSpiDevice(std::string name, uint8_t spi_bus, uint8_t spi_cs);

  /**
   * @brief: Create an actuator instance in JointController and attch it on the dcu
   * @param {string} dcu name
   * @param {CtrlChannel} dcu ctrl channel, see CtrlChannel
   * @param {ActuatorType} actuator type in ActuatorType
   * @param {string} actuator name
   * @param {string} actuator can id in canfd bus
   * @return {*}
   */
  bool AttachActuator(std::string spi_device_name, CtrlChannel ch, ActuatorType type,
                      std::string actuator_name, uint8_t id);

  /**
   * @brief: make realtime priority for ecat io thread
   * @param {int} rt_priority for thread, in range [0, 99], 99 for highest, -1 for disable
   * @param {int} bind cpu core num, -1 for disable
   * @return {*}
   */
  bool SetRealtime(int rt_priority, int bind_cpu);

  /**
   * @brief: Start the ethercat communication
   * @param {string} ifname
   * @param {uint64_t} cycle ns
   * @param {bool} enable dc
   * @return {bool} true for success
   */
  bool Start(uint64_t cycle_ns);

  /**
   * @brief: Stop the ethercat communication
   * @return {*}
   */
  void Stop();

 public:  // PowerFlowR Actuator API
  /**
   * @brief Enable all actuator
   *
   * @return true
   * @return false
   */
  bool EnableAllActuator();

  /**
   * @brief Enable an actuator
   *
   * @param  actuator name
   * @return true
   * @return false
   */
  bool EnableActuator(const std::string& name);

  /**
   * @brief Disable all actuator
   *
   * @return true
   * @return false
   */
  bool DisableAllActuator();

  /**
   * @brief Disable an actuator
   *
   * @param name
   * @return true
   * @return false
   */
  bool DisableActuator(const std::string& name);

  /**
   * @brief get tempure
   *
   * @param name
   * @return float unite Volt
   */
  float GetTempure(const std::string& name);

  /**
   * @brief Get the Power State
   *
   * @param name
   * @return ActuatorState
   */
  // ActuatorState GetPowerState(const std::string& name);

  /**
   * @brief Get the Mode
   *
   * @param name
   * @return ActuatorMode
   */
  ActuatorMode GetMode(const std::string& name);

  /**
   * @brief Get the Torque
   *        Working for PowerFlowL and OminiPicker too.
   *
   * @param name
   * @return float (Nm)
   */
  float GetEffort(const std::string& name);

  /**
   * @brief Get the Velocity
   *        Working for PowerFlowL and OminiPicker too.
   *
   * @param name
   * @return float (rad/s)
   */
  float GetVelocity(const std::string& name);

  /**
   * @brief Get the Position
   *        Working for PowerFlowL and OminiPicker too.
   *
   * @param name
   * @return float (rad)
   */
  float GetPosition(const std::string& name);

  /**
   * @brief Set the Mit Param object
   *
   * @param name
   * @param param see MitParam
   */
  void SetMitParam(const std::string& name, MitParam param);

  /**
   * @brief Set the PD cmd using MIT mode
   *        For PowerFlowL and OminiPicker, only pos and effort work, keep others as 0.
   *
   * @param name
   * @param pos (rad)
   * @param vel (rad/s)
   * @param effort (Nm)
   * @param kp
   * @param kd
   */
  void SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp, float kd);

  /**
   * @brief Reset the motor
   *
   * @param name
   * @return true
   * @return false
   */
  bool SetZeroPosition(const std::string& name);

  /**
   * @brief Get the Error Code object
   *
   * @param name
   * @return uint16_t
   */
  uint16_t GetErrorCode(const std::string& name);

 protected:
  JointController();

 private:
  std::atomic_bool is_running_{false};
  static JointController* instance_;
};

using JointControllerPtr = std::shared_ptr<JointController>;

}  // namespace xyber