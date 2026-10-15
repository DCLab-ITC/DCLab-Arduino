/**
 * @file dclab_can.h
 * @brief CAN frames of the DCLab boards: plain C99, no hardware access.
 *
 * Works with any CAN controller: Arduino + MCP2515, STM32 HAL (bxCAN/FDCAN),
 * ESP32 TWAI, ... Build a frame, then send its id/dlc/data with your CAN
 * library; pass received frames to the decode functions.
 *
 * Firmware V2 formats (Smart Driver 2.13, Motor Controller 2.22, Sensor 2.7,
 * Absolute Encoder 2.5+):
 *  - Motor command (Smart Driver and Motor Controller): 4 bytes per motor,
 *    little-endian uint32 with the mode in bits 2..0, the enable flag in
 *    bit 3 and the goal float32 in bits 31..4 (its 4 lowest mantissa bits
 *    dropped). On one receive ID, DLC 4 carries the first motor (M1), DLC 8
 *    M1 in bytes 0-3 and the second motor (M2) in 4-7; a half without the
 *    enable flag is skipped (that motor keeps its command).
 *  - Encoder feedback: float32 position, 4 bytes per encoder; two encoders
 *    sharing an ID send one DLC 8 frame (lower-numbered first), sampled at
 *    the same instant. Speed is not sent (see dclab_speed_update()).
 * The Sensor input, Controller, Swerve and IMU frames are unchanged from V1.
 */
#ifndef DCLAB_CAN_H
#define DCLAB_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;
    bool extended;
    uint8_t dlc;
    uint8_t data[8];
} dclab_can_frame_t;

/* Motor modes (bits 2..0 of a command word). */
enum {
    DCLAB_MODE_VOLTAGE = 0,        /* goal in V (Motor Controller: its motor's voltage/current) */
    DCLAB_MODE_POSITION = 1,       /* rad */
    DCLAB_MODE_SPEED = 2,          /* rad/s */
    DCLAB_MODE_POSITION_RAMP = 3,  /* goal = ramp rate, until the board restarts */
    DCLAB_MODE_SPEED_RAMP = 4,
    DCLAB_MODE_VOLTAGE_RAMP = 5
};
#define DCLAB_MOTOR_ENABLE 0x8U    /* bit 3 of a command word */

/* --- motors: Smart Driver and Motor Controller --- */
/** The motors on one receive ID: M1/M2 = the first/second motor on can_id
 * (lower-numbered first). A motor alone on its ID is M1. A disabled motor
 * keeps its command; it is not a stop (send voltage 0). */
typedef struct {
    uint16_t can_id;
    bool m1_enable;
    uint8_t mode1;
    float goal1;
    bool m2_enable;
    uint8_t mode2;
    float goal2;
} dclab_motor_command_t;
/** DLC 4 for M1 alone; DLC 8 when M2 is enabled (M1's half disabled unless
 * m1_enable). False when neither is enabled, can_id is above 0x7FF, or an
 * enabled motor has an unknown mode or a non-finite goal. */
bool dclab_motor_command(const dclab_motor_command_t *command, dclab_can_frame_t *out);
/** The enabled 4-byte word for one motor; decode returns its enable flag. */
uint32_t dclab_motor_word(uint8_t mode, float goal);
bool dclab_motor_word_decode(uint32_t word, uint8_t *mode, float *goal);

/* --- encoder feedback (Smart Driver, Sensor, Absolute Encoder) --- */
/** Positions in a feedback frame: returns 1 (DLC 4) or 2 (DLC 8, shared
 * ID; positions[0] = lower-numbered encoder), 0 if not a feedback frame. */
uint8_t dclab_encoder_positions(const dclab_can_frame_t *frame, float positions[2]);

/** Speed from successive positions (boards no longer send it). */
typedef struct {
    bool started;
    float position;
    uint32_t time_us;
    float speed;
} dclab_speed_t;
/** Update with a new position and its receive time (e.g. micros());
 * alpha 0 = raw difference, closer to 1 = smoother. Returns the speed. */
float dclab_speed_update(dclab_speed_t *s, float position, uint32_t now_us, float alpha);

/* --- Sensor board digital/analog frame (DLC 8) --- */
bool dclab_sensor_inputs(const dclab_can_frame_t *frame, float analog[5], bool digital[4]);

/* --- Controller board (values 0..1 are clamped) --- */
void dclab_controller_pwm(uint16_t can_id, const float duty[4], dclab_can_frame_t *out);
void dclab_controller_servo(uint16_t can_id, const float position[4], dclab_can_frame_t *out);
/** Bit i of digital_mask = digital output i+1, of solenoid_mask = solenoid i+1. */
void dclab_controller_digital(uint16_t can_id, uint8_t digital_mask, uint8_t solenoid_mask,
    dclab_can_frame_t *out);

/* --- Swerve module (V1 Swerve Controller only; the V2 Motor Controller uses
 *     dclab_motor_command above) --- */
void dclab_swerve_command(uint16_t can_id, float steering_rad, float wheel_rad_s, dclab_can_frame_t *out);
bool dclab_swerve_feedback(const dclab_can_frame_t *frame, float *steering_rad, float *wheel_rad_s);

/* --- IMU board --- */
bool dclab_imu_quaternion(const dclab_can_frame_t *frame, float wxyz[4]);

#ifdef __cplusplus
}
#endif

#endif
