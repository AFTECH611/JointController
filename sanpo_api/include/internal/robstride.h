#pragma once

// projects
#include "internal/actuator_base.h"

namespace xyber_robstride {

enum ActuatorControlMode : char {
  SET_MODE = 'j',
  SET_PARAMETER = 'p',
};

enum ActuatorCommunicationType : uint8_t {
  TYPE_GET_ID = 0x00,
  TYPE_MOTION_CONTROL = 0x01,
  TYPE_MOTOR_REQUEST = 0x02,
  TYPE_MOTOR_ENABLE = 0x03,
  TYPE_MOTOR_STOP = 0x04,
  TYPE_SET_POS_ZERO = 0x06,
  TYPE_CAN_ID = 0x07,
  TYPE_CONTROL_MODE = 0x12,
  TYPE_GET_SINGLE_PARAM = 0x11,
  TYPE_SET_SINGLE_PARAM = 0x12,
  TYPE_ERROR_FEEBACK = 0x15,
};

class Robstride : public xyber::Actuator {
 public:
  explicit Robstride(xyber::ActuatorType type, std::string name, uint8_t id,
                     xyber::CtrlChannel ctrl_ch, xyber::MitParam param)
      : Actuator(name, type, id, ctrl_ch), mit_param_(param) {}
  ~Robstride() {}

 public:
  // Parse feedback from motor
  virtual void ParseFeedback(uint32_t can_id, const uint8_t* data) override;
  
  virtual void RequestState(xyber::ActuatorState state);
  virtual bool Enable() override;
  virtual bool Disable() override;
  virtual bool SetZero() override;
  
  virtual void SetTorque(float cur) override;
  virtual float GetTorque() override { return torque_; }
  
  virtual void SetVelocity(float vel) override;
  virtual float GetVelocity() override { return velocity_; }
  
  virtual void SetPosition(float pos) override;
  virtual float GetPosition() override { return position_; }

  virtual void SetMitParam(xyber::MitParam param) override { mit_param_ = param; }
  virtual void SetMitCmd(float pos, float vel, float toq, float kp, float kd) override;
  virtual uint32_t GetCanId() override { return can_id_; }

 private:
  int MitFloatToUint(float x, float x_min, float x_max, int bits);
  float MitUintToFloat(int x_int, float x_min, float x_max, int bits);
  void Set_Robstride_Motor_Parameter(uint16_t Index, float value, ActuatorControlMode SET_MODE);
  void Get_Robstride_Motor_Parameter(uint16_t Index);

 private:
  xyber::MitParam mit_param_;
  uint32_t can_id_;
  uint8_t master_id_ = 0xFD;  // Default master ID
};

}  // namespace xyber_robstride