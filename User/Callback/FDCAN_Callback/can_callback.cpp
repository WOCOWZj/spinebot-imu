//
// Created by Yanhong Liu on 2025/7/19.
//

#include "can_callback.h"
#include "tim_callback.h"
#include "bsp_fdcan.h"
#include "component_bmi088.h"

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (hfdcan->Instance == hfdcan1.Instance) {
        FDCAN1_Manager_Obj.flag++;
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_Manager_Obj.RxFrame.Header, FDCAN1_Manager_Obj.RxFrame.Data);
        if(FDCAN1_Manager_Obj.RxFrame.Header.Identifier == 0x00){
//            switch (FDCAN1_Manager_Obj.RxFrame.Data[2]) {
//                case 1:
//                    IMU_SendAccel(bmi088.vector_accel_world[0],bmi088.vector_accel_world[1],bmi088.vector_accel_world[2]);
//                    break;
//                case 2:
//                    IMU_SendGyro(bmi088.vector_gyro_filtered[0],bmi088.vector_gyro_filtered[1],bmi088.vector_gyro_filtered[2]);
//                    break;
//                case 3:
//                    IMU_SendEuler(bmi088.pitch,bmi088.yaw,bmi088.roll);
//                    break;
//            }
        }
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
        CAN_bus_off_check_reset(hfdcan);
    }
}