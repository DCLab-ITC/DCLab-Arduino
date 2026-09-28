// Controller board: PWM, servo, digital outputs and solenoids.
#include <SPI.h>
#include <mcp2515.h>
#include <DCLab.h>

MCP2515 mcp2515(10);
const uint16_t CONTROLLER_ID = 600;  // the board's "Receive ID"

void send(const dclab_can_frame_t &f) {
  struct can_frame m;
  m.can_id = f.id;
  m.can_dlc = f.dlc;
  memcpy(m.data, f.data, f.dlc);
  mcp2515.sendMessage(&m);
}

void setup() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  dclab_can_frame_t f;
  const float duty[4] = {0.1f, 0.2f, 0.3f, 0.4f};  // 0..1
  dclab_controller_pwm(CONTROLLER_ID, duty, &f);
  send(f);
  const float servo[4] = {0.5f, 0.5f, 0.0f, 1.0f};  // 0..1 of the pulse range
  dclab_controller_servo(CONTROLLER_ID, servo, &f);
  send(f);
  dclab_controller_digital(CONTROLLER_ID, 0b0001 /* digital 1 */, 0b000011 /* solenoids 1, 2 */, &f);
  send(f);
  delay(20);
}
