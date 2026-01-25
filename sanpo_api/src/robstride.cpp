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
#include <iostream>

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
  can_id_ = (ActuatorCommunicationType::TYPE_MOTOR_ENABLE << 24) | (master_id_ << 8) | id_;
  //can_id_ |= 0x80000000;  // Set RTR bit for extended ID
  memset(send_buf_, 0, ACTUATOR_FRAME_SIZE);
  LOG_INFO("Motor %s enable command sent", name_.c_str());
  return true;
}

bool Robstride::Disable() {
  uint8_t clear_error = 0;
  can_id_ = (ActuatorCommunicationType::TYPE_MOTOR_ENABLE << 24) | (master_id_ << 8) | id_;
  //can_id_ |= 0x80000000;  // Set RTR bit for extended ID
  memset(send_buf_, 0, ACTUATOR_FRAME_SIZE);
  send_buf_[0] = clear_error;
  LOG_INFO("Motor %s disable command sent", name_.c_str());
  return true;
}

bool Robstride::SetZero() {
  can_id_ =
      (ActuatorCommunicationType::TYPE_SET_POS_ZERO << 24) | (master_id_ << 8) | id_;
  //can_id_ |= 0x80000000;  // Set RTR bit for extended ID
  memset(send_buf_, 0, ACTUATOR_FRAME_SIZE);
  send_buf_[0] = 1;  // Command to set current position as zero
  LOG_INFO("Motor %s set zero position command sent", name_.c_str());
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
  uint16_t toq = (recv_buf_[3] & 0xF) << 8 | recv_buf_[4];
  return MitUintToFloat(toq, mit_param_.toq_min, mit_param_.toq_max, 12);
}

void Robstride::SetVelocity(float vel) {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    cmd_type_ = ROBSTRIDE_VELOCITY_CMD;
  //    cmd_velocity_ = vel;
  //    need_send_ = true;
}

float Robstride::GetVelocity() {
  uint16_t vel = recv_buf_[2] << 4 | recv_buf_[3] >> 4;
  return MitUintToFloat(vel, mit_param_.vel_min, mit_param_.vel_max, 12);
}

void Robstride::SetPosition(float pos) {
  // std::lock_guard<std::mutex> lock(data_mutex_);
  //    cmd_type_ = ROBSTRIDE_POSITION_CMD;
  //    cmd_position_ = pos;
  //    need_send_ = true;
}

float Robstride::GetPosition() {
  uint16_t pos = recv_buf_[0] << 8 | recv_buf_[1];
  return MitUintToFloat(pos, mit_param_.pos_min, mit_param_.pos_max, 16);
}

void Robstride::SetMitCmd(float pos, float vel, float toq, float kp, float kd) {
  can_id_ = (ActuatorCommunicationType::TYPE_MOTION_CONTROL << 24) |
                      (MitFloatToUint(toq, mit_param_.toq_min, mit_param_.toq_max, 16) << 8) | id_;
  //can_id_ |= 0x80000000;  // Set RTR bit for extended ID

  int pos_tmp = MitFloatToUint(pos, mit_param_.pos_min, mit_param_.pos_max, 16);
  int vel_tmp = MitFloatToUint(vel, mit_param_.vel_min, mit_param_.vel_max, 16);
  int tor_tmp = MitFloatToUint(toq, mit_param_.toq_min, mit_param_.toq_max, 16);
  int kp_tmp = MitFloatToUint(kp, mit_param_.kp_min, mit_param_.kp_max, 16);
  int kd_tmp = MitFloatToUint(kd, mit_param_.kd_min, mit_param_.kd_max, 16);

  send_buf_[0] = (pos_tmp >> 8);
  send_buf_[1] = pos_tmp;
  send_buf_[2] = (vel_tmp >> 8);
  send_buf_[3] = vel_tmp;
  send_buf_[4] = (kp_tmp >> 8);
  send_buf_[5] = kp_tmp;
  send_buf_[6] = (kd_tmp >> 8);
  send_buf_[7] = kd_tmp;
}

}  // namespace xyber_robstride