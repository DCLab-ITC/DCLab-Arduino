/**
 * @file dclab_can.h
 * @brief CAN frames of the DCLab boards: plain C99, no hardware access.
 *
 * Works with any CAN controller: Arduino + MCP2515, STM32 HAL (bxCAN/FDCAN),
 * ESP32 TWAI, ... Build a frame, then send its id/dlc/data with your CAN
 * library; pass received frames to the decode functions.
 *
 * Firmware V2 formats (Smart Driver 2.6, Sensor 2.7, Absolute Encoder 2.5+):
 *  - Smart Driver command: 4 bytes per motor, little-endian uint32 with the
 *    mode in bits 3..0 and the goal float32 in bits 31..4 (its 4 lowest
 *    mantissa bits dropped). Motors sharing one receive ID: one DLC 8 frame,
 *    motor 1 in bytes 0-3; otherwise DLC 4 on each motor's ID.
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

/* Smart Driver modes (bits 3..0). */
enum {
    DCLAB_MODE_VOLTAGE = 0,        /* goal in V */
    DCLAB_MODE_POSITION = 1,       /* rad */
    DCLAB_MODE_SPEED = 2,          /* rad/s */
    DCLAB_MODE_POSITION_RAMP = 3,  /* goal = ramp rate, until the board restarts */
    DCLAB_MODE_SPEED_RAMP = 4,
    DCLAB_MODE_VOLTAGE_RAMP = 5
};

/* --- Smart Driver --- */
/** One motor on its own receive ID (DLC 4). False for an unknown mode or a
 * non-finite goal. */
bool dclab_smart_driver_command(uint16_t can_id, uint8_t mode, float goal, dclab_can_frame_t *out);
/** Both motors when their receive IDs are equal (DLC 8). */
bool dclab_smart_driver_pair_command(uint16_t can_id, uint8_t mode1, float goal1,
    uint8_t mode2, float goal2, dclab_can_frame_t *out);
/** The 4-byte word for one motor, and back (as the board reads it). */
uint32_t dclab_motor_word(uint8_t mode, float goal);
void dclab_motor_word_decode(uint32_t word, uint8_t *mode, float *goal);

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

/* --- Swerve module --- */
void dclab_swerve_command(uint16_t can_id, float steering_rad, float wheel_rad_s, dclab_can_frame_t *out);
bool dclab_swerve_feedback(const dclab_can_frame_t *frame, float *steering_rad, float *wheel_rad_s);

/* --- IMU board --- */
bool dclab_imu_quaternion(const dclab_can_frame_t *frame, float wxyz[4]);

/* --- boards still on V1 firmware --- */
void dclab_smart_driver_command_v1(uint16_t can_id, float goal, bool position, bool speed,
    bool voltage, bool stop, bool reset, dclab_can_frame_t *out);
bool dclab_encoder_feedback_v1(const dclab_can_frame_t *frame, float *position, float *speed);

#ifdef __cplusplus
}
#endif

#endif
