// spi_device.cpp - IMPLEMENTATION WITH ROUND-ROBIN QUEUES
// Key changes marked with // CHANGED:

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
      speed_hz_(10000000), mode_(SPI_MODE_0),
      rr_motor_id_(0) {  // CHANGED: Initialize round-robin counter
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
  // Allocate buffers for each actuator
  for (auto& [name, actr] : actuator_map_) {
    ActuatorBuffers buffers;
    memset(buffers.send, 0, 8);
    memset(buffers.recv, 0, 8);
    actuator_buffers_[name] = buffers;
    
    actr->SetDataFiled(actuator_buffers_[name].send, actuator_buffers_[name].recv);
    
    // CHANGED: Initialize per-motor queue and stats
    uint8_t motor_id = actr->GetId();
    motor_queues_[motor_id] = std::queue<CanFrame>();
    motor_stats_[motor_id] = MotorStats();
    
    LOG_DEBUG("Actuator %s (Motor ID %d) initialized with dedicated queue", 
              name.c_str(), motor_id);
  }

  LOG_INFO("SpiDevice %s initialized with %zu actuators, using ROUND-ROBIN queues", 
           name_.c_str(), actuator_map_.size());
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
  std::string name = actr->GetName();
  actuator_map_[name] = actr;
  
  // Allocate buffers
  ActuatorBuffers buffers;
  memset(buffers.send, 0, 8);
  memset(buffers.recv, 0, 8);
  actuator_buffers_[name] = buffers;
  
  actr->SetDataFiled(actuator_buffers_[name].send, actuator_buffers_[name].recv);
  
  // Map motor CAN ID to actuator
  uint8_t motor_id = actr->GetId();
  response_map_[motor_id] = name;
  
  // CHANGED: Initialize per-motor queue
  motor_queues_[motor_id] = std::queue<CanFrame>();
  motor_stats_[motor_id] = MotorStats();
  
  LOG_INFO("Actuator %s (Motor ID: 0x%02X) registered to SPI device %s with dedicated queue", 
           name.c_str(), motor_id, name_.c_str());
}

void SpiDevice::OnDataReceived(uint32_t can_id, const uint8_t* data) {
  // Extract motor ID from CAN ID
  uint8_t motor_id = (can_id >> 8) & 0xFF;
  
  // Find actuator by motor ID
  auto it = response_map_.find(motor_id);
  if (it != response_map_.end()) {
    auto actr = GetActuator(it->second);
    if (actr) {
      actr->ParseFeedback(can_id, data);
    }
  } else {
    LOG_WARN("Received data for unknown motor ID: 0x%02X (CAN ID: 0x%08X)", 
             motor_id, can_id);
  }
}

Actuator* SpiDevice::GetActuator(const std::string& name) {
  auto it = actuator_map_.find(name);
  return (it != actuator_map_.end()) ? it->second : nullptr;
}

// CHANGED: New per-motor queue implementation
void SpiDevice::QueueCommand(uint32_t can_id, const uint8_t* data) {
  if (data == nullptr) {
    LOG_ERROR("QueueCommand called with null data pointer!");
    return;
  }
  
  uint8_t motor_id = can_id & 0xFF;
  
  CanFrame frame;
  frame.can_id = can_id;
  memcpy(frame.data, data, 8);
  
  std::lock_guard<std::mutex> lock(queue_mtx_);
  
  // Queue to motor-specific queue
  motor_queues_[motor_id].push(frame);
  
  // Update statistics
  motor_stats_[motor_id].commands_queued++;
  motor_stats_[motor_id].current_queue_depth = motor_queues_[motor_id].size();
  
  if (motor_stats_[motor_id].current_queue_depth > motor_stats_[motor_id].max_queue_depth) {
    motor_stats_[motor_id].max_queue_depth = motor_stats_[motor_id].current_queue_depth;
  }
  
  LOG_DEBUG("Queue Motor %d: CAN ID 0x%08X, queue_depth=%d", 
            motor_id, can_id, motor_stats_[motor_id].current_queue_depth);
}

// CHANGED: New statistics function
void SpiDevice::PrintQueueStats() {
  std::lock_guard<std::mutex> lock(queue_mtx_);
  
  LOG_INFO("=== Queue Statistics for %s ===", name_.c_str());
  
  for (auto& [motor_id, stats] : motor_stats_) {
    float success_rate = (stats.commands_sent > 0) ? 
      (stats.commands_sent * 100.0f / stats.commands_queued) : 0.0f;
    
    LOG_INFO("Motor %d: Queued=%d, Sent=%d, Rate=%.1f%%, MaxDepth=%d, Current=%d",
             motor_id, stats.commands_queued, stats.commands_sent, 
             success_rate, stats.max_queue_depth, stats.current_queue_depth);
  }
  
  LOG_INFO("Round-robin state: last_motor_id=%d", rr_motor_id_);
}

void SpiDevice::ResetQueueStats() {
  std::lock_guard<std::mutex> lock(queue_mtx_);
  
  for (auto& [motor_id, stats] : motor_stats_) {
    stats = MotorStats();
  }
  
  LOG_INFO("Queue statistics reset for %s", name_.c_str());
}

bool SpiDevice::EnableAllActuator() {
  LOG_INFO("SpiDevice::EnableAllActuator for %s, actuators: %zu", 
           name_.c_str(), actuator_map_.size());
  
  for (const auto& [name, actr] : actuator_map_) {
    if (!EnableActuator(name)) {
      LOG_ERROR("Failed to enable %s", name.c_str());
      return false;
    }
    
    // Small delay to prevent queue burst
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

bool SpiDevice::EnableActuator(const std::string& name) {
  auto actr = GetActuator(name);
  if (!actr) {
    LOG_ERROR("Actuator %s not found", name.c_str());
    return false;
  }
  
  auto it = actuator_buffers_.find(name);
  if (it == actuator_buffers_.end()) {
    LOG_ERROR("Actuator %s has no buffers allocated!", name.c_str());
    return false;
  }

  auto& buffers = it->second;
  actr->Enable();
  QueueCommand(actr->GetCanId(), buffers.send);
  
  LOG_DEBUG("Actuator %s enable command queued (CAN ID: 0x%08X)", 
            name.c_str(), actr->GetCanId());
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

}  // namespace xyber