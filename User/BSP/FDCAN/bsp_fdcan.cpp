//
// Created by Yanhong Liu on 2025/7/19.
//
//
// Created by Yuxin Dou on 2024/12/25.
//
#include <cstring>
#include "bsp_fdcan.h"
FDCAN_Manager FDCAN1_Manager_Obj;

void FDCAN_Manager::Init(FDCAN_HandleTypeDef *_hfdcan) {
    this->hfdcan = _hfdcan;

    this->FDCAN_FilterConfig.IdType = FDCAN_STANDARD_ID;
    this->FDCAN_FilterConfig.FilterIndex = 0;
    this->FDCAN_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    this->FDCAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    this->FDCAN_FilterConfig.FilterID1 = 0x00000000;
    this->FDCAN_FilterConfig.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(this->hfdcan, &this->FDCAN_FilterConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(this->hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(this->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(this->hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(this->hfdcan) != HAL_OK) {
        Error_Handler();
    }
}

void FDCAN_Manager::Send(uint16_t ID, const uint8_t *Data, uint16_t Length) {
    this->TxFrame.hcan = this->hfdcan;
    this->TxFrame.Header.IdType = FDCAN_STANDARD_ID;
    this->TxFrame.Header.TxFrameType = FDCAN_DATA_FRAME;
    this->TxFrame.Header.DataLength = Length;
    this->TxFrame.Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    this->TxFrame.Header.BitRateSwitch = FDCAN_BRS_OFF;
    this->TxFrame.Header.FDFormat = FDCAN_CLASSIC_CAN;
    this->TxFrame.Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    this->TxFrame.Header.MessageMarker = 0;

    this->TxFrame.Header.Identifier = ID;
    uint32_t free_level = 0;
    if (this->hfdcan == &hfdcan1) {
        free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    } else if (this->hfdcan == &hfdcan2) {
        free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
    }
    if (free_level > 0) {
        auto tmp = HAL_FDCAN_AddMessageToTxFifoQ(this->hfdcan, &this->TxFrame.Header, Data);
        if (tmp != HAL_OK) {
        }
    }
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
/* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

uint8_t can1_send_data[8] = {0};
void IMU_SendGyro(float gyro_x, float gyro_y, float gyro_z) {


    // 设置数据类型标识
    can1_send_data[0] = 2;  // 陀螺仪数据类型

    // 转换并填充陀螺仪数据
    uint16_t gyro_x_int = (uint16_t)float_to_uint(gyro_x, GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    uint16_t gyro_y_int = (uint16_t)float_to_uint(gyro_y, GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    uint16_t gyro_z_int = (uint16_t)float_to_uint(gyro_z, GYRO_CAN_MIN, GYRO_CAN_MAX, 16);

    // 按协议排列数据
    can1_send_data[2] = (uint8_t)(gyro_x_int & 0xFF);
    can1_send_data[3] = (uint8_t)((gyro_x_int >> 8) & 0xFF);
    can1_send_data[4] = (uint8_t)(gyro_y_int & 0xFF);
    can1_send_data[5] = (uint8_t)((gyro_y_int >> 8) & 0xFF);
    can1_send_data[6] = (uint8_t)(gyro_z_int & 0xFF);
    can1_send_data[7] = (uint8_t)((gyro_z_int >> 8) & 0xFF);

    FDCAN1_Manager_Obj.Send(0x01,can1_send_data,8);
}

void IMU_SendEuler(float pitch, float yaw, float roll) {

    // 设置数据类型标识
    can1_send_data[0] = 3;  // 欧拉角数据类型

    // 转换并填充角度数据
    uint16_t pitch_int = (uint16_t)float_to_uint(pitch * 180.f / 3.1415926536f, PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    uint16_t yaw_int = (uint16_t)float_to_uint(yaw* 180.f / 3.1415926536f, YAW_CAN_MIN, YAW_CAN_MAX, 16);
    uint16_t roll_int = (uint16_t)float_to_uint(roll* 180.f / 3.1415926536f, ROLL_CAN_MIN, ROLL_CAN_MAX, 16);

    // 按协议排列数据
    can1_send_data[2] = (uint8_t)(pitch_int & 0xFF);   // Pitch低字节
    can1_send_data[3] = (uint8_t)((pitch_int >> 8) & 0xFF); // Pitch高字节
    can1_send_data[4] = (uint8_t)(yaw_int & 0xFF);     // Yaw低字节
    can1_send_data[5] = (uint8_t)((yaw_int >> 8) & 0xFF);  // Yaw高字节
    can1_send_data[6] = (uint8_t)(roll_int & 0xFF);    // Roll低字节
    can1_send_data[7] = (uint8_t)((roll_int >> 8) & 0xFF); // Roll高字节

    FDCAN1_Manager_Obj.Send(0x01,can1_send_data,8);
}