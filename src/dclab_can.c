/* See dclab_can.h. */
#include "dclab_can.h"

#include <string.h>

static uint32_t float_bits(float value) { uint32_t b; memcpy(&b, &value, 4); return b; }
static float bits_float(uint32_t bits) { float v; memcpy(&v, &bits, 4); return v; }
static bool finite(float v) { return v == v && v - v == 0.0f; }

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void put_f32(uint8_t *p, float v) { put_u32(p, float_bits(v)); }
static float get_f32(const uint8_t *p) { return bits_float(get_u32(p)); }

static void frame(dclab_can_frame_t *out, uint32_t id, uint8_t dlc) {
    memset(out, 0, sizeof(*out));
    out->id = id;
    out->dlc = dlc;
}

uint32_t dclab_motor_word(uint8_t mode, float goal) {
    return (float_bits(goal) & ~0xFUL) | (mode & 0xFU);
}

void dclab_motor_word_decode(uint32_t word, uint8_t *mode, float *goal) {
    if (mode) *mode = (uint8_t)(word & 0xFU);
    if (goal) *goal = bits_float(word & ~0xFUL);
}

static bool command_ok(uint8_t mode, float goal) {
    return mode <= DCLAB_MODE_VOLTAGE_RAMP && finite(goal);
}

bool dclab_smart_driver_command(uint16_t can_id, uint8_t mode, float goal, dclab_can_frame_t *out) {
    if (out == NULL || !command_ok(mode, goal)) return false;
    frame(out, can_id, 4U);
    put_u32(out->data, dclab_motor_word(mode, goal));
    return true;
}

bool dclab_smart_driver_pair_command(uint16_t can_id, uint8_t mode1, float goal1,
    uint8_t mode2, float goal2, dclab_can_frame_t *out) {
    if (out == NULL || !command_ok(mode1, goal1) || !command_ok(mode2, goal2)) return false;
    frame(out, can_id, 8U);
    put_u32(&out->data[0], dclab_motor_word(mode1, goal1));
    put_u32(&out->data[4], dclab_motor_word(mode2, goal2));
    return true;
}

uint8_t dclab_encoder_positions(const dclab_can_frame_t *f, float positions[2]) {
    if (f == NULL || positions == NULL || f->extended || (f->dlc != 4U && f->dlc != 8U)) return 0U;
    positions[0] = get_f32(&f->data[0]);
    if (f->dlc == 4U) return 1U;
    positions[1] = get_f32(&f->data[4]);
    return 2U;
}

float dclab_speed_update(dclab_speed_t *s, float position, uint32_t now_us, float alpha) {
    if (!s->started) {
        s->started = true;
        s->speed = 0.0f;
    } else {
        uint32_t dt = now_us - s->time_us;
        if (dt > 0U) {
            float raw = (position - s->position) / ((float)dt * 1e-6f);
            s->speed = alpha * s->speed + (1.0f - alpha) * raw;
        }
    }
    s->position = position;
    s->time_us = now_us;
    return s->speed;
}

bool dclab_sensor_inputs(const dclab_can_frame_t *f, float analog[5], bool digital[4]) {
    if (f == NULL || f->extended || f->dlc != 8U) return false;
    const uint8_t *d = f->data;
    uint16_t raw[5] = {
        (uint16_t)((d[0] << 4) | (d[1] >> 4)),
        (uint16_t)(((d[1] & 0x0FU) << 8) | d[2]),
        (uint16_t)((d[3] << 4) | (d[4] >> 4)),
        (uint16_t)(((d[4] & 0x0FU) << 8) | d[5]),
        (uint16_t)((d[6] << 4) | (d[7] >> 4)),
    };
    for (int i = 0; i < 5; ++i) if (analog) analog[i] = (float)raw[i] / 4095.0f;
    for (int i = 0; i < 4; ++i) if (digital) digital[i] = ((d[7] >> i) & 1U) != 0U;
    return true;
}

static void controller_values(uint16_t can_id, uint8_t kind, const float v[4], dclab_can_frame_t *out) {
    frame(out, can_id, 8U);
    for (int i = 0; i < 4; ++i) {
        float x = finite(v[i]) ? v[i] : 0.0f;
        x = x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
        uint16_t raw = (uint16_t)(x * 16383.0f + 0.5f);
        out->data[2 * i] = (uint8_t)(raw >> 8);
        out->data[2 * i + 1] = (uint8_t)raw;
    }
    out->data[0] |= (uint8_t)(kind << 6);
}

void dclab_controller_pwm(uint16_t can_id, const float duty[4], dclab_can_frame_t *out) {
    controller_values(can_id, 2U, duty, out);
}

void dclab_controller_servo(uint16_t can_id, const float position[4], dclab_can_frame_t *out) {
    controller_values(can_id, 3U, position, out);
}

void dclab_controller_digital(uint16_t can_id, uint8_t digital_mask, uint8_t solenoid_mask,
    dclab_can_frame_t *out) {
    frame(out, can_id, 8U);
    out->data[0] = 1U << 6;
    out->data[1] = digital_mask & 0x0FU;
    out->data[2] = solenoid_mask & 0x3FU;
}

void dclab_swerve_command(uint16_t can_id, float steering_rad, float wheel_rad_s, dclab_can_frame_t *out) {
    frame(out, can_id, 8U);
    put_f32(&out->data[0], steering_rad);
    put_f32(&out->data[4], wheel_rad_s);
}

bool dclab_swerve_feedback(const dclab_can_frame_t *f, float *steering_rad, float *wheel_rad_s) {
    if (f == NULL || f->extended || f->dlc != 8U) return false;
    if (steering_rad) *steering_rad = get_f32(&f->data[0]);
    if (wheel_rad_s) *wheel_rad_s = get_f32(&f->data[4]);
    return true;
}

bool dclab_imu_quaternion(const dclab_can_frame_t *f, float wxyz[4]) {
    if (f == NULL || wxyz == NULL || f->extended || f->dlc != 8U) return false;
    for (int i = 0; i < 4; ++i) {
        uint8_t hi = f->data[2 * i], lo = f->data[2 * i + 1];
        float magnitude = (float)(((hi & 0x7FU) << 8) | lo) / 32767.0f;
        wxyz[i] = (hi & 0x80U) ? -magnitude : magnitude;
    }
    return true;
}

void dclab_smart_driver_command_v1(uint16_t can_id, float goal, bool position, bool speed,
    bool voltage, bool stop, bool reset, dclab_can_frame_t *out) {
    frame(out, can_id, 8U);
    out->data[0] = (uint8_t)((position ? 1U : 0U) | (speed ? 2U : 0U) | (voltage ? 4U : 0U) |
                             (stop ? 8U : 0U) | (reset ? 16U : 0U));
    put_f32(&out->data[2], goal);
}

bool dclab_encoder_feedback_v1(const dclab_can_frame_t *f, float *position, float *speed) {
    if (f == NULL || f->extended || f->dlc != 8U) return false;
    if (position) *position = get_f32(&f->data[0]);
    if (speed) *speed = get_f32(&f->data[4]);
    return true;
}
