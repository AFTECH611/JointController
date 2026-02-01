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
  std::string name = actr->GetName();
  actuator_map_[name] = actr;
  
  // Allocate buffers immediately when registering
  ActuatorBuffers buffers;
  memset(buffers.send, 0, 8);
  memset(buffers.recv, 0, 8);
  actuator_buffers_[name] = buffers;
  
  // Set buffer pointers
  actr->SetDataFiled(actuator_buffers_[name].send, actuator_buffers_[name].recv);
  
  // Map motor CAN ID to actuator for feedback routing
  uint8_t motor_id = actr->GetId();
  response_map_[motor_id] = name;
  
  LOG_INFO("Actuator %s (Motor ID: 0x%02X) registered to SPI device %s", 
           name.c_str(), motor_id, name_.c_str());
}
void analyze_can_id(uint32_t can_id, const uint8_t* data) {
    printf("\n========== CAN ID ANALYSIS ==========\n");
    printf("Received CAN ID: 0x%08X\n\n", can_id);
    
    // Print as bytes
    printf("As bytes (little-endian layout in memory):\n");
    uint8_t* bytes = (uint8_t*)&can_id;
    printf("  [0] = 0x%02X\n", bytes[0]);
    printf("  [1] = 0x%02X\n", bytes[1]);
    printf("  [2] = 0x%02X\n", bytes[2]);
    printf("  [3] = 0x%02X\n", bytes[3]);
    
    // Try different extraction methods
    printf("\nExtraction methods:\n");
    
    uint8_t method1 = can_id & 0xFF;
    uint8_t method2 = (can_id >> 8) & 0xFF;
    uint8_t method3 = (can_id >> 16) & 0xFF;
    uint8_t method4 = (can_id >> 24) & 0xFF;
    
    printf("  Bits 0-7   (& 0xFF):          0x%02X\n", method1);
    printf("  Bits 8-15  ((>> 8) & 0xFF):   0x%02X  <-- This works for you\n", method2);
    printf("  Bits 16-23 ((>> 16) & 0xFF):  0x%02X\n", method3);
    printf("  Bits 24-31 ((>> 24) & 0xFF):  0x%02X\n", method4);
    
    // Parse as big-endian vs little-endian
    printf("\nIf interpreted as big-endian (network order):\n");
    printf("  Type:      0x%02X (bits 24-31)\n", method4);
    printf("  Reserved:  0x%02X (bits 16-23)\n", method3);
    printf("  Master:    0x%02X (bits 8-15)\n", method2);
    printf("  Motor ID:  0x%02X (bits 0-7)\n", method1);
    
    printf("\nIf interpreted as little-endian (Intel order):\n");
    printf("  Motor ID:  0x%02X (bits 24-31)\n", method4);
    printf("  Master:    0x%02X (bits 16-23)\n", method3);
    printf("  Reserved:  0x%02X (bits 8-15)\n", method2);
    printf("  Type:      0x%02X (bits 0-7)\n", method1);
    
    // Analyze data payload
    printf("\nData payload:\n");
    printf("  ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    
    // Try to parse as feedback (assuming RobStride protocol)
    printf("\nTrying to parse as RobStride feedback:\n");
    uint16_t pos_raw = (data[0] << 8) | data[1];
    uint16_t vel_raw = (data[2] << 8) | data[3];
    uint16_t torq_raw = (data[4] << 8) | data[5];
    uint16_t temp_raw = (data[6] << 8) | data[7];
    
    float position = ((float)pos_raw / 32767.0f - 1.0f) * 12.5f;
    float velocity = ((float)vel_raw / 32767.0f - 1.0f) * 44.0f;
    float torque = ((float)torq_raw / 32767.0f - 1.0f) * 14.0f;
    float temp = (float)temp_raw * 0.1f;
    
    printf("  Position: %.3f rad (raw: 0x%04X)\n", position, pos_raw);
    printf("  Velocity: %.3f rad/s (raw: 0x%04X)\n", velocity, vel_raw);
    printf("  Torque:   %.3f Nm (raw: 0x%04X)\n", torque, torq_raw);
    printf("  Temp:     %.1f C (raw: 0x%04X)\n", temp, temp_raw);
    
    printf("=====================================\n\n");
}
void SpiDevice::OnDataReceived(uint32_t can_id, const uint8_t* data) {
  static int call_count = 0;
  if (call_count < 10) {  // Only print first 10 calls
    analyze_can_id(can_id, data);
    call_count++;
  }


  // Extract motor ID from CAN ID (bits 0-7)
  uint8_t motor_id = (can_id >> 8) & 0xFF;
  
  // Find actuator by motor ID
  auto it = response_map_.find(motor_id);
  if (it != response_map_.end()) {
    auto actr = GetActuator(it->second);
    if (actr) {
      // Let actuator parse its own feedback
      actr->ParseFeedback(can_id, data);
    }
  } else {
    LOG_WARN("Received data for unknown motor ID: 0x%02X (CAN ID: 0x%08X)", motor_id, can_id);
  }
}

Actuator* SpiDevice::GetActuator(const std::string& name) {
  auto it = actuator_map_.find(name);
  return (it != actuator_map_.end()) ? it->second : nullptr;
}

void SpiDevice::QueueCommand(uint32_t can_id, const uint8_t* data) {
  if (data == nullptr) {
    LOG_ERROR("QueueCommand called with null data pointer!");
    return;
  }
  
  spi_manager::CanFrame frame;
  frame.can_id = can_id;
  memcpy(frame.data, data, 8);
  
  std::lock_guard<std::mutex> lock(queue_mtx_);
  send_queue_.push(frame);
  
  LOG_DEBUG("Command queued: CAN ID 0x%08X, queue size: %zu", can_id, send_queue_.size());
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
  if (!actr) {
    LOG_ERROR("Actuator %s not found", name.c_str());
    return false;
  }
  
  // Verify buffers exist
  auto it = actuator_buffers_.find(name);
  if (it == actuator_buffers_.end()) {
    LOG_ERROR("Actuator %s has no buffers allocated!", name.c_str());
    return false;
  }

  auto& buffers = it->second;
  actr->Enable();
  QueueCommand(actr->GetCanId(), buffers.send);
  
  LOG_DEBUG("Actuator %s enable command queued (CAN ID: 0x%08X)", name.c_str(), actr->GetCanId());
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
  return actr->GetPosition();  // Already updated by ParseFeedback
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