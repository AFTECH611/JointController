/*
 * Xyber Controller - Modified for SPI/RobStride
 * CHANGED: EtherCAT -> SPI, PowerFlow -> RobStride
 */

#include <functional>
#include <map>
#include <stdexcept>
#include <vector>

#include "internal/common_utils.h"
#include "internal/robstride.h"
#include "internal/spi_device.h"
#include "internal/spi_manager.h"
#include "internal/spi_node.h"
#include "internal/version.h"
#include "joint_controller.h"

using namespace spi_manager;

namespace xyber {

// Global variables
JointController* JointController::instance_ = nullptr;

static SpiConfig spi_config_;
static SpiManager spi_manager_;
static std::unordered_map<std::string, SpiDevice*> spi_device_map_;
static std::unordered_map<std::string, SpiDevice*> actuator_spi_device_map_;

JointController::JointController() {
  LOG_INFO("JointController created. Ver %d.%d.%d (SPI/RobStride), Compiled %s %s", MAIN_VERSION,
           SUB_VERSION, PATCH_VERSION, __DATE__, __TIME__);
}

JointController::~JointController() {
  Stop();
  spi_device_map_.clear();
  actuator_spi_device_map_.clear();
}

JointController* JointController::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new JointController();
  }
  return instance_;
}

std::string JointController::GetVersion() {
  return std::to_string(MAIN_VERSION) + "." + std::to_string(SUB_VERSION) + "." +
         std::to_string(PATCH_VERSION);
}

bool JointController::CreateSpiDevice(std::string name, uint8_t spi_bus, uint8_t spi_cs) {
  if (spi_device_map_.find(name) != spi_device_map_.end()) {
    LOG_ERROR("SPI device %s is already created, add failed.", name.c_str());
    return false;
  }
  spi_device_map_[name] = new SpiDevice(name, spi_bus, spi_cs);
  return true;
}

bool JointController::AttachActuator(std::string spi_device_name, CtrlChannel ch, ActuatorType type,
                                     std::string actuator_name, uint8_t id) {
  // check check
  if (id == 0) {
    LOG_ERROR("Actuator %s invalid id 0, attach failed.", actuator_name.c_str());
    return false;
  }
  if (spi_device_map_.find(spi_device_name) == spi_device_map_.end()) {
    LOG_ERROR("Can`t find SPI device %s, attach actuator %s failed.", spi_device_name.c_str(),
              actuator_name.c_str());
    return false;
  }
  if (actuator_spi_device_map_.find(actuator_name) != actuator_spi_device_map_.end()) {
    LOG_ERROR("Actuator %s is already attached on SPI device %s.", actuator_name.c_str(),
              actuator_spi_device_map_[actuator_name]->GetName().c_str());
    return false;
  }

  // create actuator instance based on type
  Actuator* atcr = nullptr;
  switch (type) {
    case ActuatorType::Robstride_00:
      atcr = new xyber_robstride::Robstride(type, actuator_name, id, ch,
                                            ROBSTRIDE_00_MIT_MODE_DEFAULT_PARAM);
      break;
    case ActuatorType::Robstride_02:
      atcr = new xyber_robstride::Robstride(type, actuator_name, id, ch,
                                            ROBSTRIDE_02_MIT_MODE_DEFAULT_PARAM);
      break;
    default:
      LOG_ERROR("Actuator %s type %d is not supported.", actuator_name.c_str(), (int)type);
      return false;
  }
  // attach to spi device
  LOG_INFO("Motor %s (CAN ID: 0x%02X, Channel: 0x%02X) created successfully", actuator_name.c_str(),
           id, ch);
  spi_device_map_[spi_device_name]->RegisterActuator(atcr);
  actuator_spi_device_map_[actuator_name] = spi_device_map_[spi_device_name];

  return true;
}

bool JointController::SetRealtime(int rt_priority, int bind_cpu) {
  if (is_running_) {
    LOG_WARN("Controller is running, cannot set realtime.");
    return false;
  }

  spi_config_.bind_cpu = bind_cpu;
  spi_config_.rt_priority = rt_priority;

  LOG_INFO("Realtime parameters set: priority=%d, cpu=%d", rt_priority, bind_cpu);
  return true;
}

bool JointController::Start(uint64_t cycle_ns) {
  if (is_running_) {
    LOG_WARN("Controller already running");
    return true;
  }

  // register dcu to ethercat manager
  for (const auto& spi_device : spi_device_map_) {
    spi_manager_.RegisterDevice(spi_device.second);
  }

  spi_config_.cycle_time_ns = cycle_ns;

  is_running_ = spi_manager_.Start(spi_config_);

  if (is_running_) {
    LOG_INFO("Controller started successfully on %lu Hz", 1000000000UL / cycle_ns);
  } else {
    LOG_ERROR("Controller start failed");
  }
  return is_running_;
}

void JointController::Stop() {
  if (!is_running_) return;

  spi_manager_.Stop();
  is_running_ = false;

  LOG_INFO("Controller stopped");
}

bool JointController::EnableAllActuator() {
  for (const auto& [name, spi_device] : spi_device_map_) {
    if (!spi_device->EnableAllActuator()) return false;
  }
  return true;
}

bool JointController::EnableActuator(const std::string& name) {
  auto it = actuator_spi_device_map_.find(name);
  if (it == actuator_spi_device_map_.end()) return false;

  return it->second->EnableActuator(name);
}

bool JointController::DisableAllActuator() {
  for (const auto& [name, spi_device] : spi_device_map_) {
    if (!spi_device->DisableAllActuator()) return false;
  }
  return true;
}

bool JointController::DisableActuator(const std::string& name) {
  auto it = actuator_spi_device_map_.find(name);
  if (it == actuator_spi_device_map_.end()) return false;

  return it->second->DisableActuator(name);
}

// float JointController::GetTempure(const std::string& name) {
//   auto it = actuator_spi_device_map_.find(name);
//   if (it == actuator_spi_device_map_.end()) return 0.0f;

//   return (float)it->second->GetTemperature();
// }

// ActuatorState JointController::GetPowerState(const std::string& name) {
//   auto it = actuator_spi_device_map_.find(name);
//   if (it == actuator_spi_device_map_.end()) return STATE_DISABLE;

//   return it->second->GetPowerState(name);
// }

// ActuatorMode JointController::GetMode(const std::string& name) {
//   // RobStride primarily uses MIT mode
//   return MODE_MIT;
// }

float JointController::GetEffort(const std::string& name) {
  auto it = actuator_spi_device_map_.find(name);
  if (it == actuator_spi_device_map_.end()) return 0.0f;

  return it->second->GetTorque(name);
}

float JointController::GetVelocity(const std::string& name) {
  auto it = actuator_spi_device_map_.find(name);
  if (it == actuator_spi_device_map_.end()) return 0.0f;

  return it->second->GetVelocity(name);
}

float JointController::GetPosition(const std::string& name) {
  auto it = actuator_spi_device_map_.find(name);
  if (it == actuator_spi_device_map_.end()) return 0.0f;

  return it->second->GetPosition(name);
}

void JointController::SetMitParam(const std::string& name, MitParam param) {
  auto it = actuator_spi_device_map_.find(name);
  if (it == actuator_spi_device_map_.end()) return;

  return it->second->SetMitParam(name, param);
}

void JointController::SetMitCmd(const std::string& name, float pos, float vel, float effort,
                                float kp, float kd) {
  auto it = actuator_spi_device_map_.find(name);
  if (it == actuator_spi_device_map_.end()) return;

  it->second->SetMitCmd(name, pos, vel, effort, kp, kd);
}

// Additional helper functions for RobStride

// bool JointController::ResetMotor(const std::string& name) {
//   auto it = spi_device_map_.find(name);
//   if (it == spi_device_map_.end()) {
//     LOG_ERROR("Motor %s not found", name.c_str());
//     return false;
//   }

//   return it->second->Reset();
// }
}  // namespace xyber