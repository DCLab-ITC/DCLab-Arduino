// One motor alone on its receive ID: a Smart Driver motor or a Motor
// Controller channel (Smart Driver 2.13+, Motor Controller 2.22+).
// Hardware: Arduino + MCP2515 module (library: autowp "arduino-mcp2515").
#include <SPI.h>
#include <mcp2515.h>
#include <DCLab.h>

MCP2515 mcp2515(10);  // CS pin (Uno/Nano: 10, Mega: 53)

const uint16_t MOTOR_ID = 1;  // the motor's receive ID in the Super App

void send(const dclab_can_frame_t &f) {
  struct can_frame m;
  m.can_id = f.id | (f.extended ? CAN_EFF_FLAG : 0);
  m.can_dlc = f.dlc;
  memcpy(m.data, f.data, f.dlc);
  mcp2515.sendMessage(&m);
}

// A motor alone on its ID is always M1 (DLC 4).
void command(uint8_t mode, float goal) {
  dclab_motor_command_t c = {};
  c.can_id = MOTOR_ID;
  c.m1_enable = true;
  c.mode1 = mode;
  c.goal1 = goal;
  dclab_can_frame_t f;
  if (dclab_motor_command(&c, &f)) send(f);
}

void setup() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);  // match your module's crystal
  mcp2515.setNormalMode();

  // Optional: speed ramp 10 rad/s^2 until the board restarts.
  command(DCLAB_MODE_SPEED_RAMP, 10.0f);
}

void loop() {
  // Modes: DCLAB_MODE_VOLTAGE (V), DCLAB_MODE_POSITION (rad), DCLAB_MODE_SPEED (rad/s).
  // To stop: DCLAB_MODE_VOLTAGE with goal 0.
  command(DCLAB_MODE_SPEED, 5.0f);
  delay(10);  // keep sending: the motor stops after its CAN timeout
}
