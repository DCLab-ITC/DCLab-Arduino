# DCLab Arduino / STM32 library

This library builds commands for the DCLab boards and decodes their feedback. It covers the Smart Driver, Motor Controller, Sensor, Controller, Swerve module, Absolute Encoder and IMU. It targets DCLab **V2 firmware**:

- Smart Driver 2.13+
- Motor Controller 2.22+
- Sensor 2.7+
- Absolute Encoder 2.5+

The core (`src/dclab_can.h`, `src/dclab_can.c`) is plain C99 and has no hardware code. You build a frame, then send its `id`/`dlc`/`data` with any CAN library:

- **Arduino with an MCP2515 module:** see `examples/`. The examples use the [autowp arduino-mcp2515](https://github.com/autowp/arduino-mcp2515) library.
- **STM32 (HAL, STM32CubeIDE):** copy the two `src/dclab_can.*` files into your project. `extras/stm32_hal/dclab_stm32_can.c` shows send/receive with bxCAN.
- **Other controllers (ESP32 TWAI, UNO R4, …):** the same functions work; copy the id, DLC and data into that library's frame.

## Install (Arduino IDE)

1. Sketch → Include Library → Add .ZIP Library… and pick `DCLab-Arduino.zip`, or clone this repository into `~/Arduino/libraries/DCLab`.
2. Install "arduino-mcp2515" (autowp) from the Library Manager for the examples.

## Update

Run `update.sh` (Linux, macOS) or double-click `update.cmd` (Windows) in the library's folder
(`Arduino/libraries/DCLab`). It downloads the latest version from GitHub and replaces the folder (a
git clone is pulled instead). Then restart the Arduino IDE.

## Frame formats (V2)

**Motor command (Smart Driver and Motor Controller).** `dclab_motor_command()` builds the frame for the motors on one receive ID:

```c
dclab_motor_command_t c = {0};
c.can_id = 101;
c.m1_enable = true;  c.mode1 = DCLAB_MODE_VOLTAGE; c.goal1 = 3.0f;
c.m2_enable = true;  c.mode2 = DCLAB_MODE_SPEED;   c.goal2 = -2.0f;
dclab_can_frame_t f;
if (dclab_motor_command(&c, &f)) { /* send f.id, f.dlc, f.data */ }
```

- M1 and M2 are the first and second motor on `can_id`: the lower-numbered motor (or Motor Controller channel) is M1. A motor alone on its ID is M1.
- `m1_enable` alone gives DLC 4. `m2_enable` gives DLC 8; if `m1_enable` is false, M1's half is disabled and M1 keeps its command.
- It returns false when neither motor is enabled, `can_id` is above `0x7FF`, or an enabled motor has an unknown mode or a goal that is not finite.
- If you enable M2 on an ID that has only one motor, the board ignores the frame.
- Modes:
  - `DCLAB_MODE_VOLTAGE` (V; on the Motor Controller, its motor's voltage or current)
  - `DCLAB_MODE_POSITION` (rad)
  - `DCLAB_MODE_SPEED` (rad/s)
  - `DCLAB_MODE_POSITION_RAMP`, `…_SPEED_RAMP`, `…_VOLTAGE_RAMP`: the goal is a ramp rate. It lasts until the board restarts.
- To stop a motor, send voltage 0. A disabled motor is not stopped.
- Keep sending commands: a motor stops after its CAN timeout.

On the bus, each motor takes 4 bytes: a little-endian `uint32` word. Bits 2..0 are the mode, bit 3 is the enable flag (`DCLAB_MOTOR_ENABLE`) and bits 31..4 are the goal, a `float32` without its 4 lowest mantissa bits (about 6 significant digits). DLC 4 carries M1; DLC 8 carries M1 in bytes 0-3 and M2 in bytes 4-7. A word without the enable flag is skipped.

**Encoder feedback.** This covers Smart Driver motors, Motor Controller channels, Sensor encoders and the Absolute Encoder. Each encoder sends a `float32` position in 4 bytes.

- Two encoders that share a transmit ID send one DLC 8 frame. The lower-numbered encoder comes first, and both are sampled at the same instant.
- `dclab_encoder_positions()` returns 1 or 2 positions.
- The boards no longer send speed. `dclab_speed_update()` computes it from successive positions.

**Motor Controller (V2, replaces the Swerve Controller):** use `dclab_motor_command` on each channel's receive ID (up to two channels per ID) and decode its positions with the encoder feedback functions on each channel's transmit ID. The `dclab_swerve_*` functions are for the V1 Swerve Controller only.

These formats need Smart Driver 2.13 and Motor Controller 2.22 or later. Older boards ignore these commands, and these boards ignore commands from older versions of this library, so update both together.

**Unchanged from V1:** the Sensor digital/analog frame (`dclab_sensor_inputs`), the Controller PWM, servo, digital and solenoid frames, the Swerve frames and the IMU quaternion.

## Changes from the SmartDriver library 1.0

- The library now covers every DCLab board, not just the Smart Driver, and the V2 formats.
- The core no longer depends on Arduino, so it runs on STM32 as well.
- One motor command for the Smart Driver and the Motor Controller (`dclab_motor_command`) replaces `dclab_smart_driver_command` and `dclab_smart_driver_pair_command`. The V1 functions were removed.
- The 1.0 class left `positionMode` uninitialised; the C functions here have no hidden state.

## Test

```
cc -std=c99 -Wall -Wextra -Werror -Isrc test/test_dclab_can.c src/dclab_can.c -lm && ./a.out
```
