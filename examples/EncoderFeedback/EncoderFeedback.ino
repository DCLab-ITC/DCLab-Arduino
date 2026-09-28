// Position feedback from any DCLab encoder: Smart Driver motors, Sensor
// encoders, Absolute Encoder (float32 position; 4 bytes, or 8 bytes when two
// encoders share the ID). Speed is computed here from successive positions.
#include <SPI.h>
#include <mcp2515.h>
#include <DCLab.h>

MCP2515 mcp2515(10);
const uint16_t ENCODER_ID = 101;
dclab_speed_t speeds[2];

void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  struct can_frame r;
  if (mcp2515.readMessage(&r) != MCP2515::ERROR_OK) return;
  if ((r.can_id & CAN_EFF_FLAG) || r.can_id != ENCODER_ID) return;
  dclab_can_frame_t in = {r.can_id, false, r.can_dlc, {0}};
  memcpy(in.data, r.data, r.can_dlc);
  float p[2];
  uint8_t n = dclab_encoder_positions(&in, p);
  uint32_t now = micros();
  for (uint8_t i = 0; i < n; ++i) {
    Serial.print("encoder "); Serial.print(i);
    Serial.print(" position "); Serial.print(p[i], 4);
    Serial.print(" rad, speed "); Serial.print(dclab_speed_update(&speeds[i], p[i], now, 0.8f), 3);
    Serial.println(" rad/s");
  }
}
