// spi_device.h - FIXED WITH ROUND-ROBIN QUEUE
#pragma once
#include <unordered_map>
#include <vector>
#include <deque>  // CHANGED: Use deque for round-robin
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
  
  // CRITICAL FIX: Round-robin queue management
  virtual bool HasPendingData() override {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    return !motor_queues_.empty() && !motor_queues_.begin()->second.empty();
  }

  virtual bool GetNextTxData(uint32_t& can_id, uint8_t* data) override;

  void RegisterActuator(Actuator* actr);
  Actuator* GetActuator(const std::string& name);
  virtual std::string GetName() override { return name_; }
  void SetChannelId(CtrlChannel ch, uint8_t id);
  
  virtual void OnDataReceived(uint32_t can_id, const uint8_t* data) override;

 public:  // Actuator Stuff
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

  void QueueCommand(uint32_t can_id, const uint8_t* data, uint8_t motor_id);

  void SetSpeed(uint32_t speed_hz) { speed_hz_ = speed_hz; }
  void SetMode(uint8_t mode) { mode_ = mode; }

 private:
  std::string name_;
  uint8_t bus_;
  uint8_t cs_;
  int fd_;
  uint32_t speed_hz_;
  uint8_t mode_;

  std::unordered_map<std::string, Actuator*> actuator_map_;
  std::unordered_map<CtrlChannel, std::vector<Actuator*>> ctrl_channel_map_;
  
  struct ActuatorBuffers {
    uint8_t send[8];
    uint8_t recv[8];
  };
  std::unordered_map<std::string, ActuatorBuffers> actuator_buffers_;
  
  // ============================================================
  // KEY FIX: Per-motor queues with round-robin scheduling
  // This ensures Motor ID 4 doesn't starve
  // ============================================================
  std::unordered_map<uint8_t, std::deque<spi_manager::CanFrame>> motor_queues_;
  std::vector<uint8_t> motor_order_;  // Round-robin order
  size_t next_motor_idx_ = 0;
  
  mutable std::mutex queue_mtx_;

  spi_manager::CanFrame send_buf_;
  spi_manager::CanFrame recv_buf_;
  
  std::unordered_map<uint8_t, std::string> response_map_;
};

}  // namespace xyber