/*
 * @Author: Long Vu
 * @Date:   2026-01-11 10:30:00
 * @Description: Main actuator control
 * @Version: 1.0
 * @License: MIT
 * @Copyright: (c) 2026 Long Vu
 */

// cpp
#include <cstring>

// projects
#include <mutex>
#include "common_type.h"
#include "internal/common_utils.h"
#include "internal/robstride.h"

using namespace xyber;

namespace xyber_robstride {
int Robstride::MitFloatToUint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float Robstride::MitUintToFloat(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void Robstride::RequestState(ActuatorState state) {
  send_buf_[0] = TYPE_MOTOR_ENABLE;
  send_buf_[1] = state;
}

bool Robstride::Enable() {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    cmd_type_ = ROBSTRIDE_ENABLE_CMD;
  //    need_send_ = true;
  LOG_INFO("Motor %s enable command sent", name_.c_str());
  return true;
}

bool Robstride::Disable() {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    cmd_type_ = ROBSTRIDE_RESET_CMD;
  //    need_send_ = true;
  LOG_INFO("Motor %s disable command sent", name_.c_str());
  return true;
}

bool Robstride::Reset() {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    cmd_type_ = ROBSTRIDE_RESET_CMD;
  //    need_send_ = true;
  return true;
}

bool Robstride::SetZero() {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    cmd_type_ = ROBSTRIDE_ZERO_CMD;
  //    need_send_ = true;
  return true;
}

void Robstride::SetTorque(float cur) {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    cmd_type_ = ROBSTRIDE_TORQUE_CMD;
  //    cmd_torque_ = cur;
  //    need_send_ = true;
  return;
}

float Robstride::GetTorque() {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    return feedback_torque_;
  return 0.0f;
}

void Robstride::SetVelocity(float vel) {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    cmd_type_ = ROBSTRIDE_VELOCITY_CMD;
  //    cmd_velocity_ = vel;
  //    need_send_ = true;
}

float Robstride::GetVelocity() {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    return feedback_velocity_;
  return 0.0f;
}

void Robstride::SetPosition(float pos) {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    cmd_type_ = ROBSTRIDE_POSITION_CMD;
  //    cmd_position_ = pos;
  //    need_send_ = true;
}

float Robstride::GetPosition() {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    return feedback_position_;
  return 0.0f;
}

void Robstride::Set_Robstride_Motor_Parameter(uint16_t Index, float Value,
                                              ActuatorControlMode SET_MODE) {
  send_buf_[0] = Index;
  send_buf_[1] = Index >> 8;
  send_buf_[2] = 0x00;
  send_buf_[3] = 0x00;

  if (SET_MODE == 'p') {
    memcpy(&send_buf_[4], &Value, 4);
  } else if (SET_MODE == 'j') {
    // Motor_Set_All.set_motor_mode = int(Value);
    send_buf_[4] = (uint8_t)Value;
    send_buf_[5] = 0x00;
    send_buf_[6] = 0x00;
    send_buf_[7] = 0x00;
  }
}

void Robstride::Get_Robstride_Motor_Parameter(uint16_t Index) {
  send_buf_[0] = Index;
  send_buf_[1] = Index >> 8;
  send_buf_[2] = 0x00;
  send_buf_[3] = 0x00;
  send_buf_[4] = 0x00;
  send_buf_[5] = 0x00;
  send_buf_[6] = 0x00;
  send_buf_[7] = 0x00;
}

void Robstride::SetMitCmd(float pos, float vel, float toq, float kp, float kd) {
  int pos_tmp = MitFloatToUint(pos, mit_param_.pos_min, mit_param_.pos_max, 16);
  int vel_tmp = MitFloatToUint(vel, mit_param_.vel_min, mit_param_.vel_max, 16);
  int tor_tmp = MitFloatToUint(toq, mit_param_.toq_min, mit_param_.toq_max, 16);
  int kp_tmp = MitFloatToUint(kp, mit_param_.kp_min, mit_param_.kp_max, 16);
  int kd_tmp = MitFloatToUint(kd, mit_param_.kd_min, mit_param_.kd_max, 16);

  send_buf_[0] = (pos_tmp >> 8);
  send_buf_[1] = pos_tmp & 0xFF;
  send_buf_[2] = (vel_tmp >> 8);
  send_buf_[3] = vel_tmp & 0xFF;
  send_buf_[4] = (kp_tmp >> 8);
  send_buf_[5] = kp_tmp & 0xFF;
  send_buf_[6] = (kd_tmp >> 8);
  send_buf_[7] = kd_tmp & 0xFF;
}

}  // namespace xyber_robstride