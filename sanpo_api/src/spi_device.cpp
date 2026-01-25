#include "internal/spi_device.h"
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

#include "internal/common_utils.h"

using namespace spi_manager;

namespace xyber {

SpiDevice::SpiDevice(std::string name, uint8_t bus, uint8_t cs)
    : name_(name), bus_(bus), cs_(cs), fd_(-1), 
      speed_hz_(10000000), mode_(SPI_MODE_0) {
  memset(&send_buf_, 0, sizeof(send_buf_));
  memset(&recv_buf_, 0, sizeof(recv_buf_));
  LOG_DEBUG("SpiDevice %s bus %u cs %u created.", name.c_str(), bus, cs);
}

SpiDevice::~SpiDevice() {
  for (auto& it : actuator_map_) {
    delete it.second;
  }
}

bool SpiDevice::Init() {
  // Allocate separate buffers for each actuator
  for (auto& [name, actr] : actuator_map_) {
    ActuatorBuffers buffers;
    memset(buffers.send, 0, 8);
    memset(buffers.recv, 0, 8);
    actuator_buffers_[name] = buffers;
    
    actr->SetDataFiled(actuator_buffers_[name].send, actuator_buffers_[name].recv);
    
    LOG_DEBUG("Actuator %s initialized with dedicated buffers", name.c_str());
  }

  LOG_INFO("SpiDevice %s initialized with %zu actuators", name_.c_str(), actuator_map_.size());
  return true;
}

bool SpiDevice::Open() {
  char dev_path[32];
  snprintf(dev_path, sizeof(dev_path), "/dev/spidev%d.%d", bus_, cs_);

  fd_ = open(dev_path, O_RDWR);
  if (fd_ < 0) {
    LOG_ERROR("Failed to open %s: %s", dev_path, strerror(errno));
    return false;
  }

  if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0) {
    LOG_ERROR("Failed to set SPI mode: %s", strerror(errno));
    Close();
    return false;
  }

  if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0) {
    LOG_ERROR("Failed to set SPI speed: %s", strerror(errno));
    Close();
    return false;
  }

  LOG_INFO("SPI device %s opened successfully, speed=%u Hz", dev_path, speed_hz_);
  return true;
}

void SpiDevice::Close() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool SpiDevice::Transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len) {
  if (fd_ < 0) {
    LOG_ERROR("SPI device %s not open", name_.c_str());
    return false;
  }

  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  tr.tx_buf = (unsigned long)tx_data;
  tr.rx_buf = (unsigned long)rx_data;
  tr.len = len;
  tr.speed_hz = speed_hz_;
  tr.bits_per_word = 8;

  if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
    LOG_ERROR("SPI transfer failed on %s: %s", name_.c_str(), strerror(errno));
    return false;
  }

  return true;
}

void SpiDevice::RegisterActuator(Actuator* actr) {
  actuator_map_[actr->GetName()] = actr;
  LOG_INFO("Actuator %s registered to SPI device %s", actr->GetName().c_str(), name_.c_str());
}

Actuator* SpiDevice::GetActuator(const std::string& name) {
  auto it = actuator_map_.find(name);
  return (it != actuator_map_.end()) ? it->second : nullptr;
}

void SpiDevice::QueueCommand(uint32_t can_id, const uint8_t* data) {
  spi_manager::CanFrame frame;
  frame.can_id = can_id;
  memcpy(frame.data, data, 8);
  
  std::lock_guard<std::mutex> lock(queue_mtx_);
  send_queue_.push(frame);
}

bool SpiDevice::EnableAllActuator() {
  LOG_INFO("SpiDevice::EnableAllActuator for %s, actuators: %zu", 
           name_.c_str(), actuator_map_.size());
  
  for (const auto& [name, actr] : actuator_map_) {
    LOG_INFO("  Enabling actuator: %s, ptr: %p", name.c_str(), (void*)actr);
    
    if (actr == nullptr) {
      LOG_ERROR("  Actuator %s is NULL!", name.c_str());
      return false;
    }
    
    if (!EnableActuator(name)) {
      LOG_ERROR("  Failed to enable %s", name.c_str());
      return false;
    }
  }
  return true;
}

bool SpiDevice::EnableActuator(const std::string& name) {
  auto actr = GetActuator(name);
  LOG_INFO("Enabling actuator %s on device %s", name.c_str(), name_.c_str());
  if (!actr) {
    LOG_ERROR("Actuator %s not found", name.c_str());
    return false;
  }

  auto& buffers = actuator_buffers_[name];
  actr->Enable();
  QueueCommand(actr->GetCanId(), buffers.send);
  
  LOG_DEBUG("Actuator %s enable command queued", name.c_str());
  return true;
}

bool SpiDevice::DisableAllActuator() {
  for (const auto& [name, actr] : actuator_map_) {
    if (!DisableActuator(name)) return false;
  }
  return true;
}

bool SpiDevice::DisableActuator(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) return false;

  auto& buffers = actuator_buffers_[name];
  actr->Disable();
  QueueCommand(actr->GetCanId(), buffers.send);
  
  LOG_DEBUG("Actuator %s disable command queued", name.c_str());
  return true;
}

bool SpiDevice::SetZeroPosition(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) return false;

  auto& buffers = actuator_buffers_[name];
  actr->SetZero();
  QueueCommand(actr->GetCanId(), buffers.send);
  
  LOG_INFO("Actuator %s set zero command queued", name.c_str());
  return true;
}

void SpiDevice::SetMitCmd(const std::string& name, float pos, float vel, 
                          float effort, float kp, float kd) {
  auto actr = GetActuator(name);
  if (!actr) return;

  auto& buffers = actuator_buffers_[name];
  actr->SetMitCmd(pos, vel, effort, kp, kd);
  QueueCommand(actr->GetCanId(), buffers.send);
}

float SpiDevice::GetPosition(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) return 0.0f;

  auto& buffers = actuator_buffers_[name];
  return actr->GetPosition();
}

float SpiDevice::GetVelocity(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) return 0.0f;
  return actr->GetVelocity();
}

float SpiDevice::GetTorque(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) return 0.0f;
  return actr->GetTorque();
}

void SpiDevice::SetMitParam(const std::string& name, MitParam param) {
  auto actr = GetActuator(name);
  if (!actr) return;
  actr->SetMitParam(param);
}

}