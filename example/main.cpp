#include <cstdint>
#include <iostream>
#include <thread>
#include "joint_controller.h"

using namespace xyber;
using namespace std::chrono_literals;

/**
 * Simple test to isolate Motor ID 4 behavior
 * This test ONLY controls Motor ID 4 on both SPI channels
 */
int main() {
  std::cout << "\n========================================" << std::endl;
  std::cout << "  Motor ID 4 Isolated Test" << std::endl;
  std::cout << "  Testing ONLY Motor ID 4" << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  JointControllerPtr controller(JointController::GetInstance());
  
  // Create SPI devices
  controller->CreateSpiDevice("left_leg", 0, 0);
  controller->CreateSpiDevice("right_leg", 0, 1);
  
  // ONLY attach Motor ID 4
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "motor4_left", 4);
  controller->AttachActuator("right_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "motor4_right", 4);
  
  std::cout << "Only Motor ID 4 attached on both channels\n" << std::endl;
  
  // Set realtime
  controller->SetRealtime(80, 1);
  
  // Start controller
  if (!controller->Start(1000000)) {
    std::cerr << "Failed to start" << std::endl;
    return -1;
  }
  
  std::this_thread::sleep_for(100ms);
  
  // Enable Motor ID 4
  std::cout << "Enabling Motor ID 4 (left)..." << std::endl;
  controller->EnableActuator("motor4_left");
  std::this_thread::sleep_for(20ms);
  
  std::cout << "Enabling Motor ID 4 (right)..." << std::endl;
  controller->EnableActuator("motor4_right");
  std::this_thread::sleep_for(200ms);
  
  std::cout << "\nStarting control loop (10 seconds)...\n" << std::endl;
  
  // Simple control loop - ONLY Motor ID 4
  float dt = 0.0f;
  int updates_left = 0;
  int updates_right = 0;
  float last_pos_left = -999.0f;
  float last_pos_right = -999.0f;
  
  for (int i = 0; i < 1000; i++) {  // 10 seconds at 100Hz
    double pos_cmd = 2.0 * sin(dt);
    
    // Send commands to Motor ID 4 only
    controller->SetMitCmd("motor4_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("motor4_right", -pos_cmd, 0, 0, 0.9, 0.2);
    
    // Get feedback
    float pos_left = controller->GetPosition("motor4_left");
    float pos_right = controller->GetPosition("motor4_right");
    
    // Track updates
    if (pos_left != last_pos_left) {
      updates_left++;
      last_pos_left = pos_left;
    }
    if (pos_right != last_pos_right) {
      updates_right++;
      last_pos_right = pos_right;
    }
    
    // Print every 100ms
    if (i % 10 == 0) {
      std::cout << "t=" << (i/100.0f) << "s | "
                << "Left: " << pos_left << " | "
                << "Right: " << pos_right << " | "
                << "Updates: L=" << updates_left << " R=" << updates_right 
                << std::endl;
    }
    
    dt += 0.01f;
    if (dt >= 6.28f) dt = 0.0f;
    
    std::this_thread::sleep_for(10ms);
  }
  
  // Results
  std::cout << "\n========================================" << std::endl;
  std::cout << "  Test Results" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Total iterations: 1000" << std::endl;
  std::cout << "Motor4 Left updates:  " << updates_left 
            << " (" << (updates_left/10.0f) << "%)" << std::endl;
  std::cout << "Motor4 Right updates: " << updates_right 
            << " (" << (updates_right/10.0f) << "%)" << std::endl;
  
  if (updates_left < 900 || updates_right < 900) {
    std::cout << "\n❌ ISOLATED TEST FAILED" << std::endl;
    std::cout << "Motor ID 4 has issues even when alone!" << std::endl;
    std::cout << "Possible hardware or firmware problem." << std::endl;
  } else {
    std::cout << "\n✅ ISOLATED TEST PASSED" << std::endl;
    std::cout << "Motor ID 4 works fine when alone." << std::endl;
    std::cout << "Problem is likely queue/timing with multiple motors." << std::endl;
  }
  
  // Cleanup
  controller->DisableAllActuator();
  std::this_thread::sleep_for(100ms);
  controller->Stop();
  
  return 0;
}