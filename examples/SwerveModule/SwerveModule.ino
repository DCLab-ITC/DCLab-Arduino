// Swerve module: steering position (rad) and wheel speed (rad/s), with feedback.
#include <SPI.h>
#include <mcp2515.h>
#include <DCLab.h>

MCP2515 mcp2515(10);
const uint16_t SWERVE_RX_ID = 201;  // "Receive CAN ID"
const uint16_t SWERVE_TX_ID = 251;  // "Transmit CAN ID"

void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  dclab_can_frame_t f;
  dclab_swerve_command(SWERVE_RX_ID, 1.57f, 10.0f, &f);
  struct can_frame m;
  m.can_id = f.id;
  m.can_dlc = f.dlc;
  memcpy(m.data, f.data, f.dlc);
  mcp2515.sendMessage(&m);

  struct can_frame r;
  if (mcp2515.readMessage(&r) == MCP2515::ERROR_OK && r.can_id == SWERVE_TX_ID) {
    dclab_can_frame_t in = {r.can_id, false, r.can_dlc, {0}};
    memcpy(in.data, r.data, r.can_dlc);
    float steering, wheel;
    if (dclab_swerve_feedback(&in, &steering, &wheel)) {
      Serial.print(steering); Serial.print(' '); Serial.println(wheel);
    }
  }
  delay(10);
}
