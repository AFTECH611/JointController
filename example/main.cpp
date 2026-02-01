#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>
#include "joint_controller.h"

using namespace xyber;
using namespace std::chrono_literals;

// Helper function to enable all motors with proper delay
bool EnableAllActuatorsSequentially(JointControllerPtr controller) {
  std::vector<std::string> motor_names = {
    "hip_roll_left", 
    "hip_pitch_left", 
    "knee_left", 
    "ankle_left",
    "hip_roll_right", 
    "hip_pitch_right", 
    "knee_right", 
    "ankle_right"
  };
  
  std::cout << "\n=== Enabling Motors Sequentially ===" << std::endl;
  
  for (const auto& name : motor_names) {
    std::cout << "Enabling " << name << "..." << std::endl;
    
    if (!controller->EnableActuator(name)) {
      std::cerr << "ERROR: Failed to enable " << name << std::endl;
      return false;
    }
    
    // CRITICAL: 10ms delay between each motor enable
    // This matches the Python script behavior and prevents queue overflow
    std::this_thread::sleep_for(10ms);
    
    std::cout << "  ✓ " << name << " enabled" << std::endl;
  }
  
  std::cout << "=== All Motors Enabled Successfully ===" << std::endl;
  return true;
}

int main() {
  std::cout << "\n========================================" << std::endl;
  std::cout << "  RobStride Motor Controller Demo" << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  JointControllerPtr controller(JointController::GetInstance());
  
  // ============================================================
  // STEP 1: Create SPI Devices
  // ============================================================
  std::cout << "Step 1: Creating SPI devices..." << std::endl;
  controller->CreateSpiDevice("left_leg", 0, 0);   // /dev/spidev0.0
  controller->CreateSpiDevice("right_leg", 0, 1);  // /dev/spidev0.1
  std::cout << "  ✓ SPI devices created\n" << std::endl;
  
  // ============================================================
  // STEP 2: Attach Actuators to LEFT LEG (CS 0)
  // ============================================================
  std::cout << "Step 2: Attaching actuators to LEFT LEG..." << std::endl;
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "hip_roll_left", 1);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "hip_pitch_left", 2);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "knee_left", 3);
  controller->AttachActuator("left_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "ankle_left", 4);
  std::cout << "  ✓ Left leg motors attached\n" << std::endl;
  
  // ============================================================
  // STEP 3: Attach Actuators to RIGHT LEG (CS 1)
  // ============================================================
  std::cout << "Step 3: Attaching actuators to RIGHT LEG..." << std::endl;
  controller->AttachActuator("right_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "hip_roll_right", 1);
  controller->AttachActuator("right_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "hip_pitch_right", 2);
  controller->AttachActuator("right_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "knee_right", 3);
  controller->AttachActuator("right_leg", CtrlChannel::CH1, ActuatorType::Robstride_00, 
                            "ankle_right", 4);
  std::cout << "  ✓ Right leg motors attached\n" << std::endl;
  
  // ============================================================
  // STEP 4: Set Realtime Parameters
  // ============================================================
  std::cout << "Step 4: Setting realtime parameters..." << std::endl;
  controller->SetRealtime(80, 1);
  std::cout << "  ✓ Realtime priority: 80, CPU core: 1\n" << std::endl;
  
  // ============================================================
  // STEP 5: Start Controller
  // ============================================================
  std::cout << "Step 5: Starting controller..." << std::endl;
  if (!controller->Start(1000000)) {  // 1ms cycle (1,000,000 ns)
    std::cerr << "ERROR: Failed to start controller!" << std::endl;
    return -1;
  }
  std::cout << "  ✓ Controller started at 1kHz\n" << std::endl;
  
  // Wait for controller to stabilize
  std::cout << "Waiting for controller to stabilize..." << std::endl;
  std::this_thread::sleep_for(100ms);
  
  // ============================================================
  // STEP 6: Enable All Motors WITH PROPER DELAY
  // ============================================================
  std::cout << "\nStep 6: Enabling all motors..." << std::endl;
  
  // FIXED: Use sequential enabling with 10ms delay between each motor
  // This is critical to prevent queue overflow and ensure motor ID 1 works
  if (!EnableAllActuatorsSequentially(controller)) {
    std::cerr << "ERROR: Failed to enable motors!" << std::endl;
    controller->Stop();
    return -1;
  }
  
  // Additional delay to ensure all enable commands are processed
  std::cout << "\nWaiting for enable commands to complete..." << std::endl;
  std::this_thread::sleep_for(200ms);
  std::cout << "  ✓ Ready to start control loop\n" << std::endl;
  
  // ============================================================
  // STEP 7: Control Loop
  // ============================================================
  std::cout << "\n========================================" << std::endl;
  std::cout << "  Starting Control Loop (30 seconds)" << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  float dt = 0.0f;
  const int total_iterations = 100 * 30;  // 30 seconds at 100Hz
  
  for (size_t i = 0; i < total_iterations; i++) {
    // Sinusoidal position command
    double pos_cmd = 2.0 * sin(dt);
    
    // Control LEFT leg motors
    controller->SetMitCmd("hip_roll_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("hip_pitch_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("knee_left", pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("ankle_left", pos_cmd, 0, 0, 0.9, 0.2);
    
    // Control RIGHT leg motors (opposite phase)
    controller->SetMitCmd("hip_roll_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("hip_pitch_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("knee_right", -pos_cmd, 0, 0, 0.9, 0.2);
    controller->SetMitCmd("ankle_right", -pos_cmd, 0, 0, 0.9, 0.2);
    
    // Read feedback from key motors
    float hip_roll_left_pos = controller->GetPosition("hip_roll_left");   // Motor ID 1
    float hip_pitch_left_pos = controller->GetPosition("hip_pitch_left"); // Motor ID 2
    float knee_left_pos = controller->GetPosition("knee_left");           // Motor ID 3
    float ankle_left_pos = controller->GetPosition("ankle_left");         // Motor ID 4
    
    float hip_roll_right_pos = controller->GetPosition("hip_roll_right");   // Motor ID 1
    float hip_pitch_right_pos = controller->GetPosition("hip_pitch_right"); // Motor ID 2
    float knee_right_pos = controller->GetPosition("knee_right");           // Motor ID 3
    float ankle_right_pos = controller->GetPosition("ankle_right");         // Motor ID 4
    
    // Print feedback every 1 second (every 100 iterations at 10ms loop)
    if (i % 100 == 0) {
      std::cout << "\n--- Feedback at t=" << (i/100) << "s ---" << std::endl;
      std::cout << "Left Leg:  HR=" << hip_roll_left_pos 
                << " | HP=" << hip_pitch_left_pos
                << " | K=" << knee_left_pos
                << " | A=" << ankle_left_pos << std::endl;
      std::cout << "Right Leg: HR=" << hip_roll_right_pos 
                << " | HP=" << hip_pitch_right_pos
                << " | K=" << knee_right_pos
                << " | A=" << ankle_right_pos << std::endl;
      
      // Check if Motor ID 1 is responding
      if (hip_roll_left_pos == 0.0f && hip_roll_right_pos == 0.0f && i > 200) {
        std::cout << "⚠️  WARNING: Motor ID 1 (both legs) showing zero position!" << std::endl;
      }
    }
    
    // Increment time for sinusoidal motion
    dt += 0.01f;
    if (dt >= 6.28f) {  // 2*PI
      dt = 0.0f;
    }
    
    // Sleep for 10ms (100Hz control loop)
    std::this_thread::sleep_for(10ms);
  }
  
  // ============================================================
  // STEP 8: Shutdown Sequence
  // ============================================================
  std::cout << "\n========================================" << std::endl;
  std::cout << "  Shutting Down" << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  std::cout << "Disabling all motors..." << std::endl;
  controller->DisableAllActuator();
  std::this_thread::sleep_for(100ms);
  
  std::cout << "Stopping controller..." << std::endl;
  controller->Stop();
  
  std::cout << "\n✓ Program completed successfully!" << std::endl;
  
  return 0;
}