/* Host test: cc -std=c99 -Wall -Wextra -Werror -Isrc test/test_dclab_can.c src/dclab_can.c && ./a.out */
#include "dclab_can.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    dclab_can_frame_t f;
    /* Golden vector: speed 6.0 (0x40C00000), mode 2. */
    assert(dclab_smart_driver_command(1, DCLAB_MODE_SPEED, 6.0f, &f));
    const uint8_t golden[4] = {0x02, 0x00, 0xC0, 0x40};
    assert(f.id == 1 && f.dlc == 4 && !f.extended && memcmp(f.data, golden, 4) == 0);
    assert(!dclab_smart_driver_command(1, 6, 1.0f, &f));
    assert(!dclab_smart_driver_command(1, DCLAB_MODE_SPEED, NAN, &f));

    assert(dclab_smart_driver_pair_command(1, DCLAB_MODE_VOLTAGE, 3.0f, DCLAB_MODE_POSITION, -2.0f, &f));
    assert(f.dlc == 8);
    uint8_t mode; float goal;
    uint32_t w2 = (uint32_t)f.data[4] | ((uint32_t)f.data[5] << 8) | ((uint32_t)f.data[6] << 16) | ((uint32_t)f.data[7] << 24);
    dclab_motor_word_decode(w2, &mode, &goal);
    assert(mode == DCLAB_MODE_POSITION && goal == -2.0f);
    dclab_motor_word_decode(dclab_motor_word(DCLAB_MODE_POSITION, 3.14159f), &mode, &goal);
    assert(fabsf(goal - 3.14159f) < 3.14159f * 2e-6f);

    float p[2];
    dclab_can_frame_t fb = {101, false, 8, {0}};
    float a = 1.5f, b = -2.5f; memcpy(fb.data, &a, 4); memcpy(&fb.data[4], &b, 4);
    assert(dclab_encoder_positions(&fb, p) == 2 && p[0] == 1.5f && p[1] == -2.5f);
    fb.dlc = 4; assert(dclab_encoder_positions(&fb, p) == 1 && p[0] == 1.5f);
    fb.dlc = 6; assert(dclab_encoder_positions(&fb, p) == 0);

    dclab_speed_t s = {0};
    assert(dclab_speed_update(&s, 0.0f, 1000000U, 0.0f) == 0.0f);
    assert(fabsf(dclab_speed_update(&s, 0.5f, 1100000U, 0.0f) - 5.0f) < 1e-4f);

    dclab_can_frame_t in = {500, false, 8, {200 >> 4, ((200 & 15) << 4) | (4095 >> 8), 4095 & 255,
                                             0, 0, 0, 4095 >> 4, ((4095 & 15) << 4) | 0x5}};
    float an[5]; bool dg[4];
    assert(dclab_sensor_inputs(&in, an, dg));
    assert((int)(an[0] * 4095.0f + 0.5f) == 200 && an[1] == 1.0f && an[4] == 1.0f);
    assert(dg[0] && !dg[1] && dg[2] && !dg[3]);

    const float duty[4] = {1.0f, 0.0f, 2.0f, -1.0f};
    dclab_controller_pwm(600, duty, &f);
    assert(f.data[0] >> 6 == 2 && (((f.data[0] & 0x3F) << 8) | f.data[1]) == 16383 && f.data[6] == 0);
    dclab_controller_digital(600, 0x9, 0x20, &f);
    assert(f.data[0] == 0x40 && f.data[1] == 0x9 && f.data[2] == 0x20);

    dclab_can_frame_t imu = {1000, false, 8, {0x7F, 0xFF, 0x80, 0x00, 0x40, 0x00, 0xC0, 0x00}};
    float q[4]; assert(dclab_imu_quaternion(&imu, q));
    assert(fabsf(q[0] - 1.0f) < 1e-6f && q[1] == 0.0f && q[3] < 0.0f);

    dclab_smart_driver_command_v1(1, 6.0f, false, false, true, false, false, &f);
    assert(f.dlc == 8 && f.data[0] == 4);
    puts("PASS: DCLab CAN frames");
    return 0;
}
