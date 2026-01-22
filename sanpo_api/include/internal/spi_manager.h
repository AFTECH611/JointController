// spi_manager.h
#pragma once
#include <atomic>
#include <thread>
#include <unordered_map>
#include <vector>
#include "spi_node.h"
#include "spi_protocol.h"

namespace spi_manager {

struct SpiConfig {
  uint64_t cycle_time_ns = 1000000;  // 1ms default
  int rt_priority = -1;              // No RT by default
  int bind_cpu = -1;                 // No CPU binding
};

class SpiManager {
 public:
  explicit SpiManager();
  virtual ~SpiManager();

  // Register spine boards
  void RegisterDevice(SpiNode* node);

  // Start/Stop realtime loop
  bool Start(SpiConfig cfg);
  void Stop();

  bool IsRunning() const { return is_running_; }

 private:
  // Realtime communication loop
  void WorkLoop();

  // Process one spi device
  bool ProcessBoard(SpiNode* board);

  // Thread control
  std::thread work_thread_;
  std::atomic<bool> is_running_{false};

  SpiConfig cfg_;
  spi_protocol::CRC8 crc_calculator_;
  std::unordered_map<std::string, SpiNode*> nodes_map_;
};

}  // namespace spi_manager