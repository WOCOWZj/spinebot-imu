//
// Created by Yanhong Liu on 2025/7/19.
//

#ifndef H7_IMU_CAN_CALLBACK_H
#define H7_IMU_CAN_CALLBACK_H
#include "stm32h7xx.h"
#include "bsp.h"
void CAN_bus_off_check_reset(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_ProtocolStatusTypeDef protocolStatus = {};
    HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
    if (protocolStatus.BusOff) {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}
#endif //H7_IMU_CAN_CALLBACK_H
