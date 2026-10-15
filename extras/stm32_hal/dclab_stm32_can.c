/*
 * DCLab frames on an STM32 with the HAL bxCAN driver (F1/F4/...). Copy
 * src/dclab_can.h and src/dclab_can.c into your STM32CubeIDE project, then
 * use these helpers. hcan1 must be initialised and started
 * (HAL_CAN_Start) with a filter that accepts the feedback IDs you use.
 * For FDCAN parts (G4/H7) use HAL_FDCAN_AddMessageToTxFifoQ with the same
 * id/dlc/data.
 */
#include <string.h>

#include "dclab_can.h"
#include "main.h" /* your CubeMX project: HAL and hcan1 */

extern CAN_HandleTypeDef hcan1;

HAL_StatusTypeDef dclab_can_send(const dclab_can_frame_t *f) {
    CAN_TxHeaderTypeDef h = {0};
    uint32_t mailbox;
    if (f->extended) { h.ExtId = f->id; h.IDE = CAN_ID_EXT; }
    else { h.StdId = f->id; h.IDE = CAN_ID_STD; }
    h.RTR = CAN_RTR_DATA;
    h.DLC = f->dlc;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) return HAL_BUSY;
    return HAL_CAN_AddTxMessage(&hcan1, &h, (uint8_t *)f->data, &mailbox);
}

/* Returns true and fills *f when a frame was waiting in FIFO 0. */
bool dclab_can_receive(dclab_can_frame_t *f) {
    CAN_RxHeaderTypeDef h;
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U) return false;
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &h, f->data) != HAL_OK) return false;
    f->extended = h.IDE == CAN_ID_EXT;
    f->id = f->extended ? h.ExtId : h.StdId;
    f->dlc = (uint8_t)h.DLC;
    return h.RTR == CAN_RTR_DATA;
}

/* Example: drive a motor alone on receive ID 1 (M1) at 5 rad/s and read its
 * position. */
void dclab_example_loop(void) {
    static dclab_speed_t speed;
    const dclab_motor_command_t command = {1U, true, DCLAB_MODE_SPEED, 5.0f, false, 0U, 0.0f};
    dclab_can_frame_t f;
    if (dclab_motor_command(&command, &f)) (void)dclab_can_send(&f);
    while (dclab_can_receive(&f)) {
        float p[2];
        if (f.id == 101 && dclab_encoder_positions(&f, p) >= 1) {
            float rad_s = dclab_speed_update(&speed, p[0], HAL_GetTick() * 1000U, 0.8f);
            (void)rad_s;
        }
    }
    HAL_Delay(10);
}
