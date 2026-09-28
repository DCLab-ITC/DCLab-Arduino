// Sensor board digital/analog inputs (5 analog 0..1, 4 digital).
#include <SPI.h>
#include <mcp2515.h>
#include <DCLab.h>

MCP2515 mcp2515(10);
const uint16_t INPUT_ID = 500;  // "Digital/Analog Transmit ID"

void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  struct can_frame r;
  if (mcp2515.readMessage(&r) != MCP2515::ERROR_OK || r.can_id != INPUT_ID) return;
  dclab_can_frame_t in = {r.can_id, false, r.can_dlc, {0}};
  memcpy(in.data, r.data, r.can_dlc);
  float analog[5];
  bool digital[4];
  if (!dclab_sensor_inputs(&in, analog, digital)) return;
  for (int i = 0; i < 5; ++i) { Serial.print(analog[i], 3); Serial.print(' '); }
  for (int i = 0; i < 4; ++i) { Serial.print(digital[i]); Serial.print(' '); }
  Serial.println();
}
