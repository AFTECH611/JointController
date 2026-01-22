/*
 * @Author: Long Vu
 * @Date:   2026-01-11 10:30:00
 * @Description: Base class for actuator control
 * @Version: 1.0
 * @License: MIT
 * @Copyright: (c) 2026 Long Vu
 */

#pragma once

// cpp
#include <string>

// projects
#include "common_type.h"

#define ACTUATOR_FRAME_SIZE 8

namespace xyber {

class Actuator {
 public:
  explicit Actuator(std::string name, ActuatorType type, uint8_t id, CtrlChannel ctrl_ch)
      : id_(id), type_(type), name_(name), ctrl_ch_(ctrl_ch) {}
  virtual ~Actuator() {}

  uint8_t GetId() { return id_; }
  std::string GetName() { return name_; }
  CtrlChannel GetCtrlChannel() { return ctrl_ch_; }
  ActuatorType GetType() { return type_; }
  virtual void SetDataFiled(uint8_t* send, uint8_t* recv) {
    send_buf_ = send;
    recv_buf_ = recv;
  }

 public:  // Actuator base function
  virtual void RequestState(ActuatorState state) {}
  virtual bool Enable() { return false; }
  virtual bool Disable() { return false; }
  virtual bool Reset() { return false; }
  virtual bool SetZero() { return false; }
  virtual void SetTorque(float cur) {}
  virtual float GetTorque() { return 0.0f; }
  virtual void SetVelocity(float vel) {}
  virtual float GetVelocity() { return 0.0f; }
  virtual void SetPosition(float pos) {}
  virtual float GetPosition() { return 0.0f; }

  virtual void SetMitParam(MitParam param) {}
  virtual void SetMitCmd(float pos, float vel, float toq, float kp, float kd) {}

 protected:
  uint8_t id_;
  std::string name_;
  uint8_t* send_buf_ = nullptr;
  uint8_t* recv_buf_ = nullptr;
  CtrlChannel ctrl_ch_;
  ActuatorType type_;
};

}  // namespace xyber
