#include <cstdint>
#include <iostream>
#include <thread>

#include "joint_controller.h"

using namespace xyber;
using namespace std::chrono_literals;

struct SpiDeviceInfo {
  uint8_t spi_bus;
  uint8_t spi_cs;
  std::string device_name;
};

int main() {
  JointControllerPtr controller(JointController::GetInstance());
  SpiDeviceInfo spi1 = {0, 0, "left_leg"};   // SPI bus 0, CS 0
  //SpiDeviceInfo spi2 = {0, 1, "right_leg"};  // SPI bus 0, CS 1
  controller->CreateSpiDevice(spi1.device_name, spi1.spi_bus, spi1.spi_cs);
  //controller->CreateSpiDevice(spi2.device_name, spi2.spi_bus, spi2.spi_cs);

  uint8_t actuator_can_id = 127;
  std::string actuator_name = "left_knee_motor";
  controller->AttachActuator(spi1.device_name, CtrlChannel::CH1, ActuatorType::Robstride_00,
                             actuator_name, actuator_can_id);

  controller->SetRealtime(80, 1);  // RT priority 80, bind to CPU 1
  bool ret = controller->Start(1000000);
  if (!ret) {
    std::cout << "Start Failed" << std::endl;
    return 0;
  }
  // Step 5. Control the actuator
  float dt = 0;
  float pos_begin = controller->GetPosition(actuator_name);
  for (size_t i = 0; i < 100 * 30; i++) {
    // Set target position using MIT mode
    double pos_cmd = pos_begin + 2 * sin(dt);
    controller->SetMitCmd(actuator_name, pos_cmd, 0, 0, 0.9, 0.2);

    // read current position
    float pos_now = controller->GetPosition(actuator_name);
    std::cout << "Position: Cmd " << pos_cmd << " Now " << pos_now << std::endl;

    // phase control
    dt += 0.01;
    if (dt >= 6.28) {
      dt = 0.0;
    }
    std::this_thread::sleep_for(10ms);
  }

  // Step 6. Disable the actuator and stop the controller
  controller->DisableAllActuator();
  controller->Stop();

  return 0;
}