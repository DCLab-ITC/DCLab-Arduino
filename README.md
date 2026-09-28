# DCLab Arduino / STM32 library

This library builds commands for the DCLab boards and decodes their feedback. It covers the Smart Driver, Sensor, Controller, Swerve module, Absolute Encoder and IMU. It targets DCLab **V2 firmware**:

- Smart Driver 2.6+
- Sensor 2.7+
- Absolute Encoder 2.5+

The core (`src/dclab_can.h`, `src/dclab_can.c`) is plain C99 and has no hardware code. You build a frame, then send its `id`/`dlc`/`data` with any CAN library:

- **Arduino with an MCP2515 module:** see `examples/`. The examples use the [autowp arduino-mcp2515](https://github.com/autowp/arduino-mcp2515) library.
- **STM32 (HAL, STM32CubeIDE):** copy the two `src/dclab_can.*` files into your project. `extras/stm32_hal/dclab_stm32_can.c` shows send/receive with bxCAN.
- **Other controllers (ESP32 TWAI, UNO R4, …):** the same functions work; copy the id, DLC and data into that library's frame.

## Install (Arduino IDE)

1. Sketch → Include Library → Add .ZIP Library… and pick `DCLab-Arduino.zip`, or clone this repository into `~/Arduino/libraries/DCLab`.
2. Install "arduino-mcp2515" (autowp) from the Library Manager for the examples.

## Frame formats (V2)

**Smart Driver command.** Each motor takes 4 bytes: a little-endian `uint32` word.

- Bits 3..0 are the mode:
  - `DCLAB_MODE_VOLTAGE` (V)
  - `DCLAB_MODE_POSITION` (rad)
  - `DCLAB_MODE_SPEED` (rad/s)
  - `DCLAB_MODE_POSITION_RAMP`, `…_SPEED_RAMP`, `…_VOLTAGE_RAMP`: the goal is a ramp rate. It lasts until the board restarts.
- Bits 31..4 are the goal: a `float32` without its 4 lowest mantissa bits, which leaves about 6 significant digits.
- If the two motors have **different** receive IDs, each motor's ID takes a DLC 4 frame (`dclab_smart_driver_command`).
- If they have the **same** receive ID, one DLC 8 frame carries motor 1 in bytes 0-3 and motor 2 in bytes 4-7 (`dclab_smart_driver_pair_command`).
- To stop a motor, send voltage 0.
- Keep sending commands: a motor stops after its CAN timeout.

**Encoder feedback.** This covers Smart Driver motors, Sensor encoders and the Absolute Encoder. Each encoder sends a `float32` position in 4 bytes.

- Two encoders that share a transmit ID send one DLC 8 frame. The lower-numbered encoder comes first, and both are sampled at the same instant.
- `dclab_encoder_positions()` returns 1 or 2 positions.
- The boards no longer send speed. `dclab_speed_update()` computes it from successive positions.

**Unchanged from V1:** the Sensor digital/analog frame (`dclab_sensor_inputs`), the Controller PWM, servo, digital and solenoid frames, the Swerve frames and the IMU quaternion.

**Boards still on V1 firmware:** use `dclab_smart_driver_command_v1()` and `dclab_encoder_feedback_v1()`.

## Changes from the SmartDriver library 1.0

- The library now covers every DCLab board, not just the Smart Driver, and the V2 formats.
- The core no longer depends on Arduino, so it runs on STM32 as well.
- The 1.0 class left `positionMode` uninitialised; the C functions here have no hidden state.

## Test

```
cc -std=c99 -Wall -Wextra -Werror -Isrc test/test_dclab_can.c src/dclab_can.c -lm && ./a.out
```
