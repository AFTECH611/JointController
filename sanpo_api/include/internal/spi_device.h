// spi_device.h
#pragma once
// cpp
#include <unordered_map>
#include <vector>

// projects
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

  void RegisterActuator(Actuator* actr);
  Actuator* GetActuator(const std::string& name);
  virtual std::string GetName() override { return name_; }
  void SetChannelId(CtrlChannel ch, uint8_t id);

 public:  // Actuator Stuff
  bool EnableAllActuator();
  bool EnableActuator(const std::string& name);
  bool DisableAllActuator();
  bool DisableActuator(const std::string& name);
  void SetHomingPosition(const std::string& name);
  void SaveConfig(const std::string& name);
  float GetTempure(const std::string& name);
  // ActuatorState GetPowerState(const std::string& name);

  bool SetMode(const std::string& name, ActuatorMode mode);
  ActuatorMode GetMode(const std::string& name);

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

  // Configuration
  virtual uint32_t BuildCanId(uint8_t motor_id, uint8_t cmd_type, const uint8_t* data = nullptr);
  uint8_t current_motor_id_ = 0;  // Track which motor is being controlled
  uint8_t current_cmd_type_ = 0;
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

  spi_manager::CanFrame send_buf_;
  spi_manager::CanFrame recv_buf_;
};

}  // namespace xyber