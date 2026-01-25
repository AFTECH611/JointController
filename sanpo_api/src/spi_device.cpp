/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:18:31
 */

#include "internal/spi_device.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include "common_type.h"
//#include <iostream>

// projects
#include "internal/common_utils.h"

#define COM_TIME_UP_MS 1000
#define MAX_CHANNEL_DEVICES 3
#define CHANNEL_BROADCAST_ID 0xFF

using namespace spi_manager;
using namespace std::chrono_literals;

namespace xyber {

SpiDevice::SpiDevice(std::string name, uint8_t bus, uint8_t cs)
    : fd_(-1), speed_hz_(10000000), mode_(SPI_MODE_0), SpiNode(), name_(name) {
  LOG_DEBUG("SpiDevice %s bus %u cs %u created.", name.c_str(), bus, cs);
  memset(&send_buf_, 0, sizeof(send_buf_));
  memset(&recv_buf_, 0, sizeof(recv_buf_));
}

SpiDevice::~SpiDevice() {
  for (auto& it : actuator_map_) {
    delete it.second;
  }
}

bool SpiDevice::Init() {
  for (auto& [name, actr] : actuator_map_) {
    actr->SetDataFiled(send_buf_.data, recv_buf_.data);
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

  // Set SPI mode
  if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0) {
    LOG_ERROR("Failed to set SPI mode: %s", strerror(errno));
    Close();
    return false;
  }

  LOG_INFO("SPI device %s opened, speed=%u Hz", dev_path, speed_hz_);
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
    LOG_ERROR("SPI device not open");
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
    LOG_ERROR("SPI transfer failed: %s", strerror(errno));
    return false;
  }

  return true;
}

void SpiDevice::RegisterActuator(Actuator* actr) {
  CtrlChannel ch = actr->GetCtrlChannel();
  if (ctrl_channel_map_[ch].size() >= MAX_CHANNEL_DEVICES) {
    LOG_ERROR("Channel %d has too many devices, %s register failed.", (int)ch,
              actr->GetName().c_str());
    return;
  }

  actr->SetDataFiled(send_buf_.data, recv_buf_.data);
  actuator_map_[actr->GetName()] = actr;
  ctrl_channel_map_[ch].push_back(actr);
}

Actuator* SpiDevice::GetActuator(const std::string& name) {
  auto it = actuator_map_.find(name);
  if (it != actuator_map_.end()) {
    return it->second;
  }
  return nullptr;
}

bool SpiDevice::EnableAllActuator() {
  for (const auto& [name, actr] : actuator_map_) {
    if (!EnableActuator(name)) return false;
  }
  return true;
}

bool SpiDevice::EnableActuator(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) return false;

  // if (actr->GetType() == ActuatorType::Robstride_00 ||
  //     actr->GetType() == ActuatorType::Robstride_02) {
  //   return true;
  // }

  {
    std::lock_guard<std::mutex> lock(send_mtx_);
    actr->Enable();
    send_buf_.can_id = actr->GetCanId();
  }

  LOG_DEBUG("Actuator %s %d enable success.", name.c_str(), (int)actr->GetId());

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

  {
    std::lock_guard<std::mutex> lock(send_mtx_);
    actr->Disable();
    send_buf_.can_id = actr->GetCanId();
  }

  return true;
}

// float SpiDevice::GetTempure(const std::string& name) {
//   auto actr = GetActuator(name);
//   if (!actr) return 0;

//   std::lock_guard<std::mutex> lock(recv_mtx_);
//   return actr->GetTemperature();
// }

void SpiDevice::SetTorque(const std::string& name, float cur) {
  auto actr = GetActuator(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetTorque(cur);
}

float SpiDevice::GetTorque(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) return 0;

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetTorque();
}

void SpiDevice::SetVelocity(const std::string& name, float vel) {
  auto actr = GetActuator(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetVelocity(vel);
}

float SpiDevice::GetVelocity(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) return 0;

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetVelocity();
}

void SpiDevice::SetPosition(const std::string& name, float pos) {
  auto actr = GetActuator(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetPosition(pos);
}

float SpiDevice::GetPosition(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) return 0;

  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetPosition();
}

void SpiDevice::SetMitParam(const std::string& name, MitParam param) {
  auto actr = GetActuator(name);
  if (!actr) return;

  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetMitParam(param);
}

void SpiDevice::SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp,
                          float kd) {
  auto actr = GetActuator(name);
  if (!actr) return;
  
  spi_manager::CanFrame frame;
  std::lock_guard<std::mutex> lock(send_mtx_);
  
  actr->SetMitCmd(pos, vel, effort, kp, kd);
  frame.can_id = actr->GetCanId();
  memcpy(frame.data, send_buf_.data, 8);
  
  // Thêm vào queue thay vì ghi đè
  std::lock_guard<std::mutex> queue_lock(queue_mtx_);
  send_queue_.push(frame);
                            
}

}  // namespace xyber