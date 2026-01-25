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
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float Robstride::MitUintToFloat(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void Robstride::ParseFeedback(uint32_t can_id, const uint8_t* data) {
  // Extract communication type from CAN ID (bits 24-28)
  uint8_t comm_type = (can_id >> 24) & 0x1F;
  
  // Only process motor feedback frames (TYPE_MOTOR_REQUEST = 0x02)
  if (comm_type != TYPE_MOTOR_REQUEST) {
    return;
  }
  
  // Parse feedback data (big-endian format from RobStride motor)
  // Data format: [pos_high, pos_low, vel_high, vel_low, torq_high, torq_low, temp_high, temp_low]
  uint16_t position_u16 = (data[0] << 8) | data[1];
  uint16_t velocity_u16 = (data[2] << 8) | data[3];
  uint16_t torque_u16 = (data[4] << 8) | data[5];
  uint16_t temperature_u16 = (data[6] << 8) | data[7];
  
  // Convert to physical values using RobStride protocol conversion
  // Formula from motor_cfg.cpp: ((uint16 / 32767.0) - 1.0) * max_range
  position_ = ((static_cast<float>(position_u16) / 32767.0f) - 1.0f) * mit_param_.pos_max;
  velocity_ = ((static_cast<float>(velocity_u16) / 32767.0f) - 1.0f) * mit_param_.vel_max;
  torque_ = ((static_cast<float>(torque_u16) / 32767.0f) - 1.0f) * mit_param_.toq_max;
  temperature_ = static_cast<float>(temperature_u16) * 0.1f;
  
  LOG_DEBUG("Motor %s feedback parsed: pos=%.3f rad, vel=%.3f rad/s, torq=%.3f Nm, temp=%.1f C", 
            name_.c_str(), position_, velocity_, torque_, temperature_);
}

void Robstride::RequestState(ActuatorState state) {
  send_buf_[0] = TYPE_MOTOR_ENABLE;
  send_buf_[1] = state;
}

bool Robstride::Enable() {
  can_id_ = (ActuatorCommunicationType::TYPE_MOTOR_ENABLE << 24) | (master_id_ << 8) | id_;
  memset(send_buf_, 0, ACTUATOR_FRAME_SIZE);
  LOG_INFO("Motor %s enable command prepared (CAN ID: 0x%08X)", name_.c_str(), can_id_);
  return true;
}

bool Robstride::Disable() {
  can_id_ = (ActuatorCommunicationType::TYPE_MOTOR_STOP << 24) | (master_id_ << 8) | id_;
  memset(send_buf_, 0, ACTUATOR_FRAME_SIZE);
  send_buf_[0] = 0;  // clear_error = 0
  LOG_INFO("Motor %s disable command prepared (CAN ID: 0x%08X)", name_.c_str(), can_id_);
  return true;
}

bool Robstride::SetZero() {
  can_id_ = (ActuatorCommunicationType::TYPE_SET_POS_ZERO << 24) | (master_id_ << 8) | id_;
  memset(send_buf_, 0, ACTUATOR_FRAME_SIZE);
  send_buf_[0] = 1;  // Command to set current position as zero
  LOG_INFO("Motor %s set zero position command prepared (CAN ID: 0x%08X)", name_.c_str(), can_id_);
  return true;
}

void Robstride::SetTorque(float cur) {
  // TODO: Implement torque control mode if needed
  return;
}

void Robstride::SetVelocity(float vel) {
  // TODO: Implement velocity control mode if needed
}

void Robstride::SetPosition(float pos) {
  // TODO: Implement position control mode if needed
}

void Robstride::SetMitCmd(float pos, float vel, float toq, float kp, float kd) {
  // Build CAN ID with communication type and torque in bits 8-23
  can_id_ = (ActuatorCommunicationType::TYPE_MOTION_CONTROL << 24) |
            (MitFloatToUint(toq, mit_param_.toq_min, mit_param_.toq_max, 16) << 8) | id_;

  // Convert parameters to uint16 using MIT protocol
  int pos_tmp = MitFloatToUint(pos, mit_param_.pos_min, mit_param_.pos_max, 16);
  int vel_tmp = MitFloatToUint(vel, mit_param_.vel_min, mit_param_.vel_max, 16);
  int kp_tmp = MitFloatToUint(kp, mit_param_.kp_min, mit_param_.kp_max, 16);
  int kd_tmp = MitFloatToUint(kd, mit_param_.kd_min, mit_param_.kd_max, 16);

  // Pack data in big-endian format
  send_buf_[0] = (pos_tmp >> 8);
  send_buf_[1] = pos_tmp;
  send_buf_[2] = (vel_tmp >> 8);
  send_buf_[3] = vel_tmp;
  send_buf_[4] = (kp_tmp >> 8);
  send_buf_[5] = kp_tmp;
  send_buf_[6] = (kd_tmp >> 8);
  send_buf_[7] = kd_tmp;
  
  LOG_DEBUG("Motor %s MIT cmd: pos=%.3f, vel=%.3f, toq=%.3f, kp=%.2f, kd=%.2f", 
            name_.c_str(), pos, vel, toq, kp, kd);
}

}  // namespace xyber_robstride