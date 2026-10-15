// Two motors on ONE receive ID (both Smart Driver motors, or two Motor
// Controller channels): M1 = the lower-numbered motor, M2 = the other.
// Both enabled: one 8-byte frame. m2_enable alone: 8 bytes with M1's half
// disabled, so M1 keeps its command. m1_enable alone: 4 bytes.
// Their position feedback shares one ID too (8 bytes, motor 1 first) when
// both transmit IDs are equal.
#include <SPI.h>
#include <mcp2515.h>
#include <DCLab.h>

MCP2515 mcp2515(10);
const uint16_t MOTORS_ID = 1;
const uint16_t FEEDBACK_ID = 101;
dclab_speed_t speed1, speed2;

void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  dclab_motor_command_t c = {};
  c.can_id = MOTORS_ID;
  c.m1_enable = true;
  c.mode1 = DCLAB_MODE_SPEED;
  c.goal1 = 5.0f;
  c.m2_enable = true;
  c.mode2 = DCLAB_MODE_POSITION;
  c.goal2 = 3.14f;
  dclab_can_frame_t f;
  if (dclab_motor_command(&c, &f)) {
    struct can_frame m;
    m.can_id = f.id;
    m.can_dlc = f.dlc;
    memcpy(m.data, f.data, f.dlc);
    mcp2515.sendMessage(&m);
  }

  struct can_frame r;
  while (mcp2515.readMessage(&r) == MCP2515::ERROR_OK) {
    if ((r.can_id & CAN_EFF_FLAG) || r.can_id != FEEDBACK_ID) continue;
    dclab_can_frame_t in = {r.can_id, false, r.can_dlc, {0}};
    memcpy(in.data, r.data, r.can_dlc);
    float p[2];
    if (dclab_encoder_positions(&in, p) == 2) {
      uint32_t now = micros();
      Serial.print(p[0]); Serial.print(' ');
      Serial.print(dclab_speed_update(&speed1, p[0], now, 0.8f)); Serial.print(' ');
      Serial.print(p[1]); Serial.print(' ');
      Serial.println(dclab_speed_update(&speed2, p[1], now, 0.8f));
    }
  }
  delay(10);
}
