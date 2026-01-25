#pragma once

// cpp
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>

// projects
#include "internal/common_utils.h"

namespace spi_manager {

#pragma pack(1)
// Communication buffers
struct CanFrame {
  uint32_t can_id;  // Motor ID control byte
  uint8_t data[8];  // CAN data
};

#pragma pack()

class SpiNode {
 public:
  explicit SpiNode() = default;
  virtual ~SpiNode() = default;
  virtual bool Init() { return true; }
  virtual CanFrame& GetSendBuf() = 0;
  virtual CanFrame& GetRecvBuf() = 0;
  virtual std::string GetName() { return node_name_; }

  // Provide default implementations for these virtual functions
  virtual bool Open() { return true; }
  virtual void Close() {}
  virtual bool Transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len) { return false; }

  virtual bool HasPendingData() { return false; }
  virtual bool GetNextTxData(uint32_t& can_id, uint8_t* data) { return false; }

  virtual void GetTxData(uint32_t& can_id, uint8_t* data) {
    std::lock_guard<std::mutex> lock(send_mtx_);
    can_id = GetSendBuf().can_id;
    std::memcpy(data, GetSendBuf().data, 8);
  }

  virtual void PushRxData(uint32_t can_id, const uint8_t* data) {
    std::lock_guard<std::mutex> lock(recv_mtx_);
    GetRecvBuf().can_id = can_id;
    std::memcpy(GetRecvBuf().data, data, 8);
  }

 protected:
  std::string node_name_;
  std::mutex send_mtx_, recv_mtx_;

  uint16_t send_size_ = 0;
  uint16_t recv_size_ = 0;
};

}  // namespace spi_manager