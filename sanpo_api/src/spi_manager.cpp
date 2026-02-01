// spi_manager.cpp - FIXED VERSION
#include "internal/spi_manager.h"
#include <chrono>
#include <cstring>
#include "internal/common_utils.h"
#include <iostream>
#include <iomanip>

using namespace std::chrono_literals;

namespace spi_manager {

SpiManager::SpiManager() : crc_calculator_(0x07) {}

SpiManager::~SpiManager() { Stop(); }

void SpiManager::RegisterDevice(SpiNode* node) { nodes_map_[node->GetName()] = node; }

bool SpiManager::Start(SpiConfig cfg) {
  if (is_running_) {
    LOG_WARN("SpiManager already running");
    return true;
  }

  cfg_ = cfg;
  if (nodes_map_.empty()) {
    LOG_ERROR("No slave nodes registered.");
    return false;
  }

  // Open all SPI devices
  for (auto& [id, node] : nodes_map_) {
    if (!node->Open()) {
      LOG_ERROR("Failed to open SPI device for board %s", node->GetName().c_str());
      return false;
    }

    // Initialize board
    if (!node->Init()) {
      LOG_ERROR("Failed to initialize board %s", node->GetName().c_str());
      return false;
    }
  }

  // Start realtime loop
  is_running_ = true;
  work_thread_ = std::thread(&SpiManager::WorkLoop, this);
  // wait for threads up
  std::this_thread::sleep_for(50ms);

  LOG_INFO("SpiManager started, cycle_time=%lu ns", cfg_.cycle_time_ns);
  return true;
}

void SpiManager::Stop() {
  if (!is_running_) return;

  is_running_ = false;

  if (work_thread_.joinable()) {
    work_thread_.join();
  }

  // Close all devices
  for (auto& [id, node] : nodes_map_) {
    node->Close();
    delete node;
  }
  LOG_INFO("SpiManager stopped");
}

void SpiManager::WorkLoop() {
  LOG_INFO("SPI WorkLoop started");

  if (cfg_.rt_priority >= 0) {
    xyber_utils::SetRealTimeThread(pthread_self(), "spi_io_loop", cfg_.rt_priority, cfg_.bind_cpu);
  }

  auto next_cycle = std::chrono::steady_clock::now();

  while (is_running_) {
    // ============================================================
    // KEY FIX: Process ALL devices in PARALLEL per cycle
    // This ensures Motor ID 4 on both CS pins gets equal treatment
    // ============================================================
    for (const auto& [id, node] : nodes_map_) {
      ProcessBoard(node);
    }

    next_cycle += std::chrono::nanoseconds(cfg_.cycle_time_ns);
    std::this_thread::sleep_until(next_cycle);
  }

  LOG_INFO("SPI WorkLoop stopped");
}

// ============================================================
// CRITICAL FIX: Process ONE command per motor per cycle
// Instead of draining queue, we round-robin across motors
// ============================================================
bool SpiManager::ProcessBoard(SpiNode* node) {
  // CHANGED: Process exactly ONE frame per cycle
  // This ensures all motors get equal update rate
  
  if (!node->HasPendingData()) {
    return false;  // No data to send
  }

  spi_protocol::SpiCanFrame tx_frame;
  memset(&tx_frame, 0, sizeof(tx_frame));
  
  // Build TX frame
  memcpy(tx_frame.header, spi_protocol::FRAME_HEADER, 2);
  memcpy(tx_frame.footer, spi_protocol::FRAME_FOOTER, 2);
  
  uint32_t can_id;
  if (!node->GetNextTxData(can_id, tx_frame.data)) {
    return false;
  }
  
  // Convert CAN ID to big-endian for SPI transmission
  tx_frame.can_id = __builtin_bswap32(can_id);
  tx_frame.reserved[3] = 0x10;
  tx_frame.crc = crc_calculator_.Calculate((uint8_t*)&tx_frame, 
                                           spi_protocol::SPI_FRAME_SIZE - 1);
  
  // Perform SPI transfer
  spi_protocol::SpiCanFrame rx_frame;
  if (!node->Transfer((uint8_t*)&tx_frame, (uint8_t*)&rx_frame, 
                      spi_protocol::SPI_FRAME_SIZE)) {
    LOG_ERROR("SPI transfer failed for node %s", node->GetName().c_str());
    return false;
  }
  
  // Verify response header and parse feedback
  if (memcmp(rx_frame.header, spi_protocol::FRAME_HEADER, 2) == 0) {
    // Convert CAN ID back to host byte order
    uint32_t rx_can_id = __builtin_bswap32(rx_frame.can_id);
    
    // Push received data to node (this will trigger ParseFeedback via OnDataReceived)
    node->PushRxData(rx_can_id, rx_frame.data);
    
    LOG_DEBUG("RX from %s: CAN ID 0x%08X, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
              node->GetName().c_str(), rx_can_id,
              rx_frame.data[0], rx_frame.data[1], rx_frame.data[2], rx_frame.data[3],
              rx_frame.data[4], rx_frame.data[5], rx_frame.data[6], rx_frame.data[7]);
  } else {
    LOG_WARN("Invalid frame header received from %s", node->GetName().c_str());
  }
  
  return true;
}

}  // namespace spi_manager