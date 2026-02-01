// spi_device.h - WITH PER-MOTOR ROUND-ROBIN QUEUES
// This is the BEST solution for Motor ID 4 issue

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
  
  // NEW: Per-motor queue implementation
  virtual bool HasPendingData() override {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    
    // Check if ANY motor has pending commands
    for (const auto& [motor_id, queue] : motor_queues_) {
      if (!queue.empty()) return true;
    }
    return false;
  }

  // NEW: Round-robin dequeue
  virtual bool GetNextTxData(uint32_t& can_id, uint8_t* data) override {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    
    // Try each motor in round-robin order
    for (int attempts = 0; attempts < 4; attempts++) {
      // Increment round-robin counter (1-4)
      rr_motor_id_ = (rr_motor_id_ % 4) + 1;
      
      auto it = motor_queues_.find(rr_motor_id_);
      if (it != motor_queues_.end() && !it->second.empty()) {
        auto& queue = it->second;
        auto frame = queue.front();
        queue.pop();
        
        can_id = frame.can_id;
        memcpy(data, frame.data, 8);
        
        // Update statistics
        motor_stats_[rr_motor_id_].commands_sent++;
        
        LOG_DEBUG("RR: Motor %d, queue_left=%zu, total_sent=%d", 
                  rr_motor_id_, queue.size(), 
                  motor_stats_[rr_motor_id_].commands_sent);
        
        return true;
      }
    }
    
    // No commands in any queue
    return false;
  }

  void RegisterActuator(Actuator* actr);
  Actuator* GetActuator(const std::string& name);
  virtual std::string GetName() override { return name_; }
  void SetChannelId(CtrlChannel ch, uint8_t id);
  
  virtual void OnDataReceived(uint32_t can_id, const uint8_t* data) override;

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
  
  // NEW: Queue statistics
  void PrintQueueStats();
  void ResetQueueStats();

 private:
  virtual bool Init() override;
  virtual spi_manager::CanFrame& GetSendBuf() override { return send_buf_; }
  virtual spi_manager::CanFrame& GetRecvBuf() override { return recv_buf_; }

  // NEW: Per-motor queue implementation
  void QueueCommand(uint32_t can_id, const uint8_t* data);

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
  
  // NEW: Per-motor queues instead of single queue
  std::unordered_map<uint8_t, std::queue<spi_manager::CanFrame>> motor_queues_;
  std::mutex queue_mtx_;
  
  // NEW: Round-robin state
  uint8_t rr_motor_id_ = 0;  // Last processed motor ID (0-4)
  
  // NEW: Statistics per motor
  struct MotorStats {
    int commands_queued = 0;
    int commands_sent = 0;
    int max_queue_depth = 0;
    int current_queue_depth = 0;
  };
  std::unordered_map<uint8_t, MotorStats> motor_stats_;

  // Temporary buffers for SpiNode interface
  spi_manager::CanFrame send_buf_;
  spi_manager::CanFrame recv_buf_;
  
  // Response mapping: Motor ID -> actuator name
  std::unordered_map<uint8_t, std::string> response_map_;
};

}  // namespace xyber