// spi_manager.h - OPTIMIZED DUAL-THREAD VERSION
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
  int rt_priority = -1;
  int bind_cpu = -1;
};

class SpiManager {
 public:
  explicit SpiManager();
  virtual ~SpiManager();

  void RegisterDevice(SpiNode* node);
  
  bool Start(SpiConfig cfg);
  void Stop();

  bool IsRunning() const { return is_running_; }

 private:
  // ============================================================
  // OPTIMIZATION: Per-device thread for parallel processing
  // ============================================================
  struct DeviceThread {
    std::thread thread;
    std::atomic<bool> running{false};
    SpiNode* node;
    uint64_t cycle_time_ns;
    int rt_priority;
    int bind_cpu;
    std::string name;
    
    // Performance tracking
    uint64_t cycle_count{0};
    uint64_t transfer_errors{0};
  };
  
  // Worker loop for each device
  void DeviceWorkerLoop(DeviceThread* dev_thread);
  
  // Process one device
  bool ProcessBoard(SpiNode* board);

  // Device threads (one per SPI CS)
  std::unordered_map<std::string, DeviceThread> device_threads_;
  
  // Global control
  std::atomic<bool> is_running_{false};
  SpiConfig cfg_;
  
  spi_protocol::CRC8 crc_calculator_;
  std::unordered_map<std::string, SpiNode*> nodes_map_;
};

}  // namespace spi_manager