#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include "joint_controller.h"

using namespace xyber;
using namespace std::chrono_literals;

// Helper: Enable all motors sequentially with delay
bool EnableAllActuatorsSequentially(JointControllerPtr controller) {
  std::vector<std::string> motor_names = {
    "hip_roll_left", "hip_pitch_left", "knee_left", "ankle_left",
    "hip_roll_right", "hip_pitch_right", "knee_right", "ankle_right"
  };
  
  std::cout << "\n=== Enabling Motors Sequentially ===" << std::endl;
  
  for (const auto& name : motor_names) {
    std::cout << "Enabling " << name << "..." << std::endl;
    
    if (!controller->EnableActuator(name)) {
      std::cerr << "ERROR: Failed to enable " << name << std::endl;
      return false;
    }
    
    std::this_thread::sleep_for(10ms);
    std::cout << "  ✓ " << name << " enabled" << std::endl;
  }
  
  std::cout << "=== All Motors Enabled ===" << std::endl;
  return true;
}

int main() {
  std::cout << "\n========================================" << std::endl;
  std::cout << "  RobStride Motor Controller" << std::endl;
  std::cout << "  WITH ROUND-ROBIN PER-MOTOR QUEUES" << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  JointControllerPtr controller(JointController::GetInstance());
  
  // Create SPI devices
  std::cout << "Creating SPI devices..." << std::endl;
  controller->CreateSpiDevice("left_leg", 0, 0);
  controller->CreateSpiDevice("right_leg", 0, 1);
  std::cout << "  ✓ SPI devices created\n" << std::endl;
  
  // Attach actuators
  std::cout << "Attaching actuators..." << std::endl;
  
  // Left leg
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "hip_roll_left", 1);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "hip_pitch_left", 2);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "knee_left", 3);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "ankle_left", 4);
  
  // Right leg
  controller->AttachActuator("right_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "hip_roll_right", 1);
  controller->AttachActuator("right_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "hip_pitch_right", 2);
  controller->AttachActuator("right_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "knee_right", 3);
  controller->AttachActuator("right_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "ankle_right", 4);
  
  std::cout << "  ✓ 8 actuators attached (4 per leg)\n" << std::endl;
  
  // Set realtime
  controller->SetRealtime(80, 1);
  
  // Start controller at 1kHz (no need for 2kHz with round-robin!)
  std::cout << "Starting controller at 1kHz..." << std::endl;
  if (!controller->Start(1000000)) {
    std::cerr << "ERROR: Failed to start controller!" << std::endl;
    return -1;
  }
  std::cout << "  ✓ Controller started\n" << std::endl;
  
  std::this_thread::sleep_for(100ms);
  
  // Enable motors
  std::cout << "\nEnabling all motors..." << std::endl;
  if (!EnableAllActuatorsSequentially(controller)) {
    std::cerr << "ERROR: Failed to enable motors!" << std::endl;
    controller->Stop();
    return -1;
  }
  
  std::this_thread::sleep_for(200ms);
  std::cout << "\n✓ Ready to start control loop\n" << std::endl;
  
  // ============================================================
  // Control Loop - NO STAGGERING NEEDED with round-robin!
  // ============================================================
  std::cout << "\n========================================" << std::endl;
  std::cout << "  Control Loop (30 seconds)" << std::endl;
  std::cout << "  Testing Round-Robin Fairness" << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  float dt = 0.0f;
  const int total_iterations = 3000;  // 30 seconds at 100Hz
  
  // Track each motor's update performance
  struct MotorPerformance {
    int updates = 0;
    float last_pos = -999.0f;
    std::string name;
  };
  
  std::map<std::string, MotorPerformance> perf_tracker;
  std::vector<std::string> motor_names = {
    "hip_roll_left", "hip_pitch_left", "knee_left", "ankle_left",
    "hip_roll_right", "hip_pitch_right", "knee_right", "ankle_right"
  };
  
  for (const auto& name : motor_names) {
    perf_tracker[name].name = name;
  }
  
  auto start_time = std::chrono::steady_clock::now();
  
  for (int i = 0; i < total_iterations; i++) {
    auto iter_start = std::chrono::steady_clock::now();
    
    double pos_cmd = 2.0 * sin(dt);
    
    // Send commands - NO STAGGERING! Round-robin handles fairness
    controller->SetMitCmd("hip_roll_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("hip_pitch_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("knee_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("ankle_left", pos_cmd, 0, 0, 0.9, 0.2);
    
    controller->SetMitCmd("hip_roll_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("hip_pitch_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("knee_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("ankle_right", -pos_cmd, 0, 0, 0.9, 0.2);
    
    // Read feedback and track updates
    for (const auto& name : motor_names) {
      float pos = controller->GetPosition(name);
      
      if (pos != perf_tracker[name].last_pos) {
        perf_tracker[name].updates++;
        perf_tracker[name].last_pos = pos;
      }
    }
    
    // Print statistics every second
    if (i % 100 == 0 && i > 0) {
      auto now = std::chrono::steady_clock::now();
      float elapsed = std::chrono::duration<float>(now - start_time).count();
      
      std::cout << "\n--- Statistics at t=" << (int)elapsed << "s ---" << std::endl;
      
      // Left leg
      std::cout << "Left Leg:" << std::endl;
      for (int m = 0; m < 4; m++) {
        auto& perf = perf_tracker[motor_names[m]];
        float rate = (perf.updates * 100.0f) / i;
        std::cout << "  M" << (m+1) << " (" << perf.name << "): "
                  << "pos=" << perf.last_pos 
                  << " | updates=" << perf.updates 
                  << " (" << rate << "%)" << std::endl;
      }
      
      // Right leg
      std::cout << "Right Leg:" << std::endl;
      for (int m = 4; m < 8; m++) {
        auto& perf = perf_tracker[motor_names[m]];
        float rate = (perf.updates * 100.0f) / i;
        std::cout << "  M" << (m-3) << " (" << perf.name << "): "
                  << "pos=" << perf.last_pos 
                  << " | updates=" << perf.updates 
                  << " (" << rate << "%)" << std::endl;
      }
      
      // Check Motor4 performance
      float m4_left_rate = (perf_tracker["ankle_left"].updates * 100.0f) / i;
      float m4_right_rate = (perf_tracker["ankle_right"].updates * 100.0f) / i;
      
      if (m4_left_rate < 90.0f || m4_right_rate < 90.0f) {
        std::cout << "⚠️  WARNING: Motor4 update rate below 90%!" << std::endl;
      } else {
        std::cout << "✅ Motor4 performance good (>90% update rate)" << std::endl;
      }
      
      // TODO: Print queue statistics from SPI device
      // This would require adding API to JointController to access SpiDevice
    }
    
    dt += 0.01f;
    if (dt >= 6.28f) dt = 0.0f;
    
    // Maintain 10ms loop period
    auto iter_end = std::chrono::steady_clock::now();
    auto iter_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      iter_end - iter_start);
    
    auto sleep_time = 10000us - iter_duration;
    if (sleep_time.count() > 0) {
      std::this_thread::sleep_for(sleep_time);
    }
  }
  
  // ============================================================
  // Final Performance Report
  // ============================================================
  std::cout << "\n========================================" << std::endl;
  std::cout << "  Final Performance Report" << std::endl;
  std::cout << "========================================" << std::endl;
  
  std::cout << "\nUpdate Rates:" << std::endl;
  std::cout << "Motor | Name              | Updates | Rate" << std::endl;
  std::cout << "------|-------------------|---------|------" << std::endl;
  
  bool all_good = true;
  for (size_t i = 0; i < motor_names.size(); i++) {
    auto& perf = perf_tracker[motor_names[i]];
    float rate = (perf.updates * 100.0f) / total_iterations;
    
    std::cout << "M" << ((i % 4) + 1) << "    | " 
              << std::left << std::setw(17) << perf.name << " | "
              << std::right << std::setw(7) << perf.updates << " | "
              << std::setw(5) << std::fixed << std::setprecision(1) << rate << "%";
    
    if (rate < 90.0f) {
      std::cout << " ❌";
      all_good = false;
    } else {
      std::cout << " ✅";
    }
    std::cout << std::endl;
  }
  
  std::cout << "\n";
  if (all_good) {
    std::cout << "✅ ALL MOTORS PERFORMING WELL (>90% update rate)" << std::endl;
    std::cout << "   Round-robin queues working as expected!" << std::endl;
  } else {
    std::cout << "❌ SOME MOTORS UNDERPERFORMING (<90% update rate)" << std::endl;
    std::cout << "   Check queue statistics and SPI timing" << std::endl;
  }
  
  // Check fairness - all motors should have similar rates
  std::vector<float> rates;
  for (const auto& name : motor_names) {
    float rate = (perf_tracker[name].updates * 100.0f) / total_iterations;
    rates.push_back(rate);
  }
  
  float min_rate = *std::min_element(rates.begin(), rates.end());
  float max_rate = *std::max_element(rates.begin(), rates.end());
  float variance = max_rate - min_rate;
  
  std::cout << "\nFairness Analysis:" << std::endl;
  std::cout << "  Min rate: " << min_rate << "%" << std::endl;
  std::cout << "  Max rate: " << max_rate << "%" << std::endl;
  std::cout << "  Variance: " << variance << "%" << std::endl;
  
  if (variance < 5.0f) {
    std::cout << "  ✅ Excellent fairness (<5% variance)" << std::endl;
  } else if (variance < 10.0f) {
    std::cout << "  ⚠️  Good fairness (5-10% variance)" << std::endl;
  } else {
    std::cout << "  ❌ Poor fairness (>10% variance)" << std::endl;
  }
  
  // Shutdown
  std::cout << "\n========================================" << std::endl;
  std::cout << "  Shutting Down" << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  controller->DisableAllActuator();
  std::this_thread::sleep_for(100ms);
  controller->Stop();
  
  std::cout << "\n✓ Program completed successfully!" << std::endl;
  
  return 0;
}