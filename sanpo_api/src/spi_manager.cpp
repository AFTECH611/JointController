// spi_manager.cpp
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
  // nodes_map_.clear();
  LOG_INFO("SpiManager stopped");
}

void SpiManager::WorkLoop() {
  LOG_INFO("SPI WorkLoop started");

  if (cfg_.rt_priority >= 0) {
    xyber_utils::SetRealTimeThread(pthread_self(), "spi_io_loop", cfg_.rt_priority, cfg_.bind_cpu);
  }

  auto next_cycle = std::chrono::steady_clock::now();

  while (is_running_) {
    for (const auto& [id, node] : nodes_map_) {
      ProcessBoard(node);
    }

    next_cycle += std::chrono::nanoseconds(cfg_.cycle_time_ns);
    std::this_thread::sleep_until(next_cycle);
  }

  LOG_INFO("SPI WorkLoop stopped");
}

void DumpSpiCanFrame(const spi_protocol::SpiCanFrame& f) {
  std::cout << "===== SpiCanFrame Dump =====" << std::endl;

  std::cout << "Header: ";
  for (int i = 0; i < 2; ++i) {
    std::cout << "0x"
              << std::hex
              << static_cast<int>(f.header[i])
              << " ";
  }
  std::cout << std::dec << std::endl;

  std::cout << "CAN ID (raw BE): 0x"
            << std::hex
            << std::setw(8)
            << std::setfill('0')
            << f.can_id
            << std::dec
            << std::endl;

  std::cout << "Data: ";
  for (int i = 0; i < 8; ++i) {
    std::cout << std::hex
              << std::setw(2)
              << std::setfill('0')
              << static_cast<int>(f.data[i])
              << " ";
  }
  std::cout << std::dec << std::endl;

  std::cout << "Footer: ";
  for (int i = 0; i < 2; ++i) {
    std::cout << std::hex
              << static_cast<int>(f.footer[i])
              << " ";
  }
  std::cout << std::dec << std::endl;

  std::cout << "Reserved: ";
  for (int i = 0; i < 4; ++i) {
    std::cout << std::hex
              << static_cast<int>(f.reserved[i])
              << " ";
  }
  std::cout << std::dec << std::endl;

  std::cout << "CRC8: 0x"
            << std::hex
            << static_cast<int>(f.crc)
            << std::dec
            << std::endl;

  std::cout << "--------------------------------" << std::endl;

}

bool SpiManager::ProcessBoard(SpiNode* node) {
  // Gửi tất cả lệnh trong queue
  while (node->HasPendingData()) {
    spi_protocol::SpiCanFrame tx_frame;
    memset(&tx_frame, 0, sizeof(tx_frame));
    
    memcpy(tx_frame.header, spi_protocol::FRAME_HEADER, 2);
    memcpy(tx_frame.footer, spi_protocol::FRAME_FOOTER, 2);
    
    uint32_t can_id;
    if (!node->GetNextTxData(can_id, tx_frame.data)) break;
    
    tx_frame.can_id = __builtin_bswap32(can_id);
    tx_frame.reserved[3] = 0x10;
    tx_frame.crc = crc_calculator_.Calculate((uint8_t*)&tx_frame, spi_protocol::SPI_FRAME_SIZE - 1);
    
    spi_protocol::SpiCanFrame rx_frame;
    if (!node->Transfer((uint8_t*)&tx_frame, (uint8_t*)&rx_frame, spi_protocol::SPI_FRAME_SIZE)) {
      continue;
    }
    
    // Xử lý response...
  }
  return true;
}

}  // namespace spi_manager