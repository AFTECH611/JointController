// ============================================================
// USAGE EXAMPLES: Dual-Thread vs Batch Mode
// ============================================================

#include "joint_controller.h"
#include <chrono>
#include <iostream>

using namespace xyber;
using namespace std::chrono_literals;

// ============================================================
// EXAMPLE 1: DUAL-THREAD MODE (Recommended)
// ============================================================
void example_dual_thread() {
  std::cout << "=== DUAL-THREAD MODE ===" << std::endl;
  
  JointControllerPtr controller(JointController::GetInstance());
  
  // Setup (same as before)
  controller->CreateSpiDevice("left_leg", 0, 0);
  controller->CreateSpiDevice("right_leg", 0, 1);
  
  // Attach motors...
  // (code omitted for brevity)
  
  controller->SetRealtime(80, 1);
  controller->Start(1000000);  // 1kHz
  
  // Enable motors
  std::this_thread::sleep_for(100ms);
  controller->EnableAllActuator();  // Sẽ được xử lý song song bởi 2 threads
  std::this_thread::sleep_for(200ms);
  
  // Control loop - KHÔNG CẦN THAY ĐỔI!
  // Mỗi SetMitCmd() tự động được queue và xử lý bởi thread tương ứng
  for (int i = 0; i < 1000; i++) {
    float pos_cmd = 2.0 * sin(i * 0.01);
    
    // Commands cho CS0 được xử lý bởi Thread 0
    controller->SetMitCmd("hip_roll_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("hip_pitch_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("knee_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("ankle_left", pos_cmd, 0, 0, 0.9, 0.2);
    
    // Commands cho CS1 được xử lý bởi Thread 1 (PARALLEL!)
    controller->SetMitCmd("hip_roll_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("hip_pitch_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("knee_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("ankle_right", -pos_cmd, 0, 0, 0.9, 0.2);
    
    // Get feedback
    float pos_left = controller->GetPosition("ankle_left");
    float pos_right = controller->GetPosition("ankle_right");
    
    if (i % 100 == 0) {
      std::cout << "t=" << i << ": Left=" << pos_left 
                << ", Right=" << pos_right << std::endl;
    }
    
    std::this_thread::sleep_for(10ms);
  }
  
  controller->DisableAllActuator();
  controller->Stop();
  
  std::cout << "Dual-thread mode completed!" << std::endl;
}

// ============================================================
// EXAMPLE 2: BATCH MODE (Ultra-low latency)
// ============================================================
void example_batch_mode() {
  std::cout << "=== BATCH MODE ===" << std::endl;
  
  // IMPORTANT: Requires modified SpiManager that calls SendBatch()
  // instead of ProcessBoard() in worker loop
  
  JointControllerPtr controller(JointController::GetInstance());
  
  // Setup
  controller->CreateSpiDevice("left_leg", 0, 0);
  controller->CreateSpiDevice("right_leg", 0, 1);
  
  // Attach motors...
  
  controller->SetRealtime(80, 1);
  controller->Start(1000000);  // 1kHz
  
  std::this_thread::sleep_for(100ms);
  controller->EnableAllActuator();
  std::this_thread::sleep_for(200ms);
  
  // Control loop
  for (int i = 0; i < 1000; i++) {
    float pos_cmd = 2.0 * sin(i * 0.01);
    
    // Queue all commands
    controller->SetMitCmd("hip_roll_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("hip_pitch_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("knee_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("ankle_left", pos_cmd, 0, 0, 0.9, 0.2);
    
    controller->SetMitCmd("hip_roll_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("hip_pitch_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("knee_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("ankle_right", -pos_cmd, 0, 0, 0.9, 0.2);
    
    // Batch mode worker threads will send all 4 motors per device
    // in one burst (~84µs total) instead of spreading across 4 cycles
    
    // Get feedback (updated immediately after batch send)
    float pos_left = controller->GetPosition("ankle_left");
    float pos_right = controller->GetPosition("ankle_right");
    
    if (i % 100 == 0) {
      std::cout << "t=" << i << ": Left=" << pos_left 
                << ", Right=" << pos_right << std::endl;
    }
    
    std::this_thread::sleep_for(10ms);
  }
  
  controller->DisableAllActuator();
  controller->Stop();
  
  std::cout << "Batch mode completed!" << std::endl;
}

// ============================================================
// EXAMPLE 3: LATENCY BENCHMARKING
// ============================================================
void benchmark_latency() {
  std::cout << "=== LATENCY BENCHMARK ===" << std::endl;
  
  JointControllerPtr controller(JointController::GetInstance());
  
  // Setup...
  controller->CreateSpiDevice("left_leg", 0, 0);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "motor1", 1);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "motor2", 2);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "motor3", 3);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "motor4", 4);
  
  controller->SetRealtime(80, 1);
  controller->Start(1000000);
  
  std::this_thread::sleep_for(100ms);
  controller->EnableAllActuator();
  std::this_thread::sleep_for(200ms);
  
  // Measure command-to-feedback latency
  struct MotorLatency {
    std::chrono::steady_clock::time_point cmd_time;
    std::chrono::steady_clock::time_point fb_time;
    float latency_ms;
    bool updated;
  };
  
  std::unordered_map<std::string, MotorLatency> latencies = {
    {"motor1", {}}, {"motor2", {}}, {"motor3", {}}, {"motor4", {}}
  };
  
  for (int i = 0; i < 100; i++) {
    // Send commands and timestamp
    auto t0 = std::chrono::steady_clock::now();
    
    controller->SetMitCmd("motor1", 1.0, 0, 0, 0.9, 0.2);
    latencies["motor1"].cmd_time = t0;
    latencies["motor1"].updated = false;
    
    controller->SetMitCmd("motor2", 1.0, 0, 0, 0.9, 0.2);
    latencies["motor2"].cmd_time = t0;
    latencies["motor2"].updated = false;
    
    controller->SetMitCmd("motor3", 1.0, 0, 0, 0.9, 0.2);
    latencies["motor3"].cmd_time = t0;
    latencies["motor3"].updated = false;
    
    controller->SetMitCmd("motor4", 1.0, 0, 0, 0.9, 0.2);
    latencies["motor4"].cmd_time = t0;
    latencies["motor4"].updated = false;
    
    // Poll for feedback
    std::this_thread::sleep_for(5ms);
    
    for (auto& [name, lat] : latencies) {
      float old_pos = controller->GetPosition(name);
      std::this_thread::sleep_for(100us);
      float new_pos = controller->GetPosition(name);
      
      if (new_pos != old_pos && !lat.updated) {
        lat.fb_time = std::chrono::steady_clock::now();
        lat.latency_ms = std::chrono::duration<float, std::milli>(
          lat.fb_time - lat.cmd_time).count();
        lat.updated = true;
      }
    }
    
    // Print results
    if (i % 10 == 0) {
      printf("Iteration %d latencies (ms):\n", i);
      printf("  M1: %.2f | M2: %.2f | M3: %.2f | M4: %.2f\n",
             latencies["motor1"].latency_ms,
             latencies["motor2"].latency_ms,
             latencies["motor3"].latency_ms,
             latencies["motor4"].latency_ms);
    }
    
    std::this_thread::sleep_for(10ms);
  }
  
  // Calculate statistics
  float sum = 0, max = 0, min = 1000;
  for (const auto& [name, lat] : latencies) {
    sum += lat.latency_ms;
    if (lat.latency_ms > max) max = lat.latency_ms;
    if (lat.latency_ms < min) min = lat.latency_ms;
  }
  float avg = sum / latencies.size();
  
  std::cout << "\n=== LATENCY STATISTICS ===" << std::endl;
  std::cout << "Average: " << avg << " ms" << std::endl;
  std::cout << "Min:     " << min << " ms" << std::endl;
  std::cout << "Max:     " << max << " ms" << std::endl;
  std::cout << "Range:   " << (max - min) << " ms" << std::endl;
  
  controller->DisableAllActuator();
  controller->Stop();
}

// ============================================================
// MAIN
// ============================================================
int main() {
  std::cout << "Choose mode:\n";
  std::cout << "1. Dual-Thread (Recommended)\n";
  std::cout << "2. Batch Mode (Ultra-low latency)\n";
  std::cout << "3. Latency Benchmark\n";
  std::cout << "Enter choice: ";
  
  int choice;
  std::cin >> choice;
  
  switch (choice) {
    case 1:
      example_dual_thread();
      break;
    case 2:
      example_batch_mode();
      break;
    case 3:
      benchmark_latency();
      break;
    default:
      std::cout << "Invalid choice!" << std::endl;
  }
  
  return 0;
}

// ============================================================
// EXPECTED RESULTS
// ============================================================

/*
DUAL-THREAD MODE:
------------------
Latency per motor:
- Motor 1 (CS0): 0.0 - 0.5 ms  (thread 0, cycle 0)
- Motor 2 (CS0): 0.5 - 1.0 ms  (thread 0, cycle 1)
- Motor 3 (CS0): 1.0 - 1.5 ms  (thread 0, cycle 2)
- Motor 4 (CS0): 1.5 - 2.0 ms  (thread 0, cycle 3)

- Motor 1 (CS1): 0.0 - 0.5 ms  (thread 1, cycle 0, PARALLEL!)
- Motor 2 (CS1): 0.5 - 1.0 ms  (thread 1, cycle 1, PARALLEL!)
- Motor 3 (CS1): 1.0 - 1.5 ms  (thread 1, cycle 2, PARALLEL!)
- Motor 4 (CS1): 1.5 - 2.0 ms  (thread 1, cycle 3, PARALLEL!)

Max latency: 2.0ms (giảm 33% so với 3ms single-thread)
Jitter: ±1.5ms (giảm 50% so với ±3ms)

BATCH MODE:
-----------
Latency per motor (per device):
- Motor 1: ~21 µs
- Motor 2: ~42 µs
- Motor 3: ~63 µs
- Motor 4: ~84 µs

Max latency: 84µs (GIẢM 97% so với 3ms!)
Jitter: ±84µs
Update frequency: 1kHz (tăng 4x!)

PYTHON BASELINE:
----------------
Max latency: ~168µs (parallel send to both CS pins)
Update frequency: 100Hz (sleep 10ms)

=> BATCH MODE C++ ≈ PYTHON PERFORMANCE! 🎉
*/