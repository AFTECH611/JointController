#include <cstdint>
#include <iostream>
#include <thread>
#include "joint_controller.h"

using namespace xyber;
using namespace std::chrono_literals;

int main() {
  JointControllerPtr controller(JointController::GetInstance());
  
  // Create TWO SPI devices
  controller->CreateSpiDevice("left_leg", 0, 0);   // /dev/spidev0.0
  controller->CreateSpiDevice("right_leg", 0, 1);  // /dev/spidev0.1
  
  // Attach actuators to FIRST SPI device
  controller->AttachActuator("left_leg", CtrlChannel::CH1, 
                            ActuatorType::Robstride_00, "left_knee", 127);
  
  // Attach actuators to SECOND SPI device
  controller->AttachActuator("right_leg", CtrlChannel::CH1, 
                            ActuatorType::Robstride_00, "right_knee", 127);

  

  controller->SetRealtime(80, 1);
  
  if (!controller->Start(1000000)) {  // 1ms cycle
    std::cout << "Start Failed" << std::endl;
    return -1;
  }
  

  controller->EnableAllActuator();
  // Control loop
  float dt = 0;
  for (size_t i = 0; i < 100 * 30; i++) {
    double pos_cmd = 2 * sin(dt);
    
    // Control left leg
    controller->SetMitCmd("left_knee", pos_cmd, 0, 0, 0.9, 0.2);
    
    // Control right leg
    controller->SetMitCmd("right_knee", -pos_cmd, 0, 0, 0.9, 0.2);
    
    // Read feedback
    float left_pos = controller->GetPosition("left_knee");
    float right_pos = controller->GetPosition("right_knee");
    
    std::cout << "Left: " << left_pos << " | Right: " << right_pos << std::endl;
    
    dt += 0.01;
    if (dt >= 6.28) dt = 0.0;
    
    std::this_thread::sleep_for(10ms);
  }
  
  controller->DisableAllActuator();
  controller->Stop();
  
  return 0;
}