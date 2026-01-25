#pragma once
#include <unordered_map>
#include <vector>
#include <queue>
#include <mutex>

#include "common_type.h"
#include "internal/actuator_base.h"
#include "internal/spi_node.h"

namespace xyber {

class SpiDevice : public spi_manager::SpiNode {
 public:
  explicit SpiDevice(std::string name, uint8_t bus, uint8_t cs);
  virtual ~SpiDevice();

  bool IsOpen() const { return fd_ >= 0; }

  virtual bool Open() override;
  virtual void Close() override;
  virtual bool Transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len) override;
  
  virtual bool HasPendingData() override {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    return !send_queue_.empty();
  }

  virtual bool GetNextTxData(uint32_t& can_id, uint8_t* data) override {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    if (send_queue_.empty()) return false;
    
    auto frame = send_queue_.front();
    send_queue_.pop();
    can_id = frame.can_id;
    memcpy(data, frame.data, 8);
    return true;
  }

  void RegisterActuator(Actuator* actr);
  Actuator* GetActuator(const std::string& name);
  virtual std::string GetName() override { return name_; }

 public:  // Actuator API
  bool EnableAllActuator();
  bool EnableActuator(const std::string& name);
  bool DisableAllActuator();
  bool DisableActuator(const std::string& name);
  bool SetZeroPosition(const std::string& name);

  void SetTorque(const std::string& name, float cur);
  float GetTorque(const std::string& name);
  void SetVelocity(const std::string& name, float vel);
  float GetVelocity(const std::string& name);
  void SetPosition(const std::string& name, float pos);
  float GetPosition(const std::string& name);

  void SetMitParam(const std::string& name, MitParam param);
  void SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp, float kd);

 private:
  virtual bool Init() override;
  virtual spi_manager::CanFrame& GetSendBuf() override { return send_buf_; }
  virtual spi_manager::CanFrame& GetRecvBuf() override { return recv_buf_; }

  // Helper: Queue a command for transmission
  void QueueCommand(uint32_t can_id, const uint8_t* data);

 private:
  std::string name_;
  uint8_t bus_;
  uint8_t cs_;
  int fd_;
  uint32_t speed_hz_;
  uint8_t mode_;

  std::unordered_map<std::string, Actuator*> actuator_map_;
  
  // Separate buffers for each actuator to avoid conflicts
  struct ActuatorBuffers {
    uint8_t send[8];
    uint8_t recv[8];
  };
  std::unordered_map<std::string, ActuatorBuffers> actuator_buffers_;
  
  std::queue<spi_manager::CanFrame> send_queue_;
  std::mutex queue_mtx_;

  // Temporary buffers for SpiNode interface
  spi_manager::CanFrame send_buf_;
  spi_manager::CanFrame recv_buf_;
  
  // Response mapping: CAN ID -> actuator name
  std::unordered_map<uint32_t, std::string> response_map_;
  std::mutex response_mtx_;
};

}